extends Node2D
class_name WaterGrid

@export var columns := 40
@export var rows := 24
@export var cell_size := 24.0
@export var goal_required_ratio := 0.55
@export var simulation_steps := 4
@export var lateral_flow_factor := 0.15
@export var upward_flow_factor := 0.35
@export var splash_chunk_mass := 0.08
@export var occupancy_displacement_factor := 0.92
@export var displacement_search_radius := 5
@export var surface_follow_speed := 2.6
@export var surface_smoothing_passes := 3
@export var hydrostatic_relaxation_speed := 7.5
@export var hydrostatic_mass_threshold := 0.02
@export var wave_tension := 28.0
@export var wave_spread := 18.0
@export var wave_damping := 8.5
@export var wave_substeps := 4
@export var wave_impact_strength := 190.0
@export var carve_strength_threshold := 0.3

const MAX_MASS := 1.0
const MAX_COMPRESS := 0.25
const MIN_MASS := 0.0001
const MIN_FLOW := 0.0001
const MAX_FLOW := 1.0

var masses: Array[PackedFloat32Array] = []
var solids: Array[PackedByteArray] = []
var diggable: Array[PackedByteArray] = []
var body_occupancy: Array[PackedFloat32Array] = []
var collision_rects: Array[Rect2] = []
var goal_rect := Rect2i(31, 19, 7, 4)
var scan_left_to_right := true
var rendered_surface_y := PackedFloat32Array()
var rendered_surface_target_y := PackedFloat32Array()
var rendered_surface_velocity := PackedFloat32Array()
var rendered_surface_min_y := PackedFloat32Array()
var rendered_surface_max_y := PackedFloat32Array()
var rendered_surface_valid := PackedByteArray()
var queued_surface_impacts: Array = []


func _ready() -> void:
	reset_level()


func _physics_process(_delta: float) -> void:
	_apply_body_constraints()
	for _step in range(simulation_steps):
		_simulate_step()
	_relax_hydrostatic_basins(_delta)
	_update_visual_surface(_delta)
	queue_redraw()
	_clear_dynamic_inputs()


func reset_level() -> void:
	_init_arrays()
	_build_level()
	_rebuild_collision_rects()
	_initialize_visual_surface()
	queued_surface_impacts.clear()
	queue_redraw()


func get_playfield_size() -> Vector2:
	return Vector2(columns * cell_size, rows * cell_size)


func get_collision_rects() -> Array[Rect2]:
	return collision_rects.duplicate()


func register_circle_body(world_center: Vector2, radius: float) -> void:
	var local_center := to_local(world_center)
	var min_cell := _world_to_cell(local_center - Vector2(radius, radius))
	var max_cell := _world_to_cell(local_center + Vector2(radius, radius))

	for x in range(max(0, min_cell.x), min(columns, max_cell.x + 1)):
		for y in range(max(0, min_cell.y), min(rows, max_cell.y + 1)):
			if solids[x][y] != 0:
				continue

			var occupancy := _sample_circle_cell_occupancy(local_center, radius, x, y)
			if occupancy <= 0.0:
				continue

			body_occupancy[x][y] = minf(1.0, body_occupancy[x][y] + occupancy)


func queue_impact(world_center: Vector2, radius: float, intensity: float) -> void:
	if intensity <= 0.0:
		return
	var impact := {
		"center": world_center,
		"radius": radius,
		"intensity": intensity
	}
	queued_surface_impacts.append(impact)


func carve_circle(world_center: Vector2, radius: float) -> bool:
	var local_center := to_local(world_center)
	var min_cell := _world_to_cell(local_center - Vector2(radius, radius))
	var max_cell := _world_to_cell(local_center + Vector2(radius, radius))
	var changed := false

	for x in range(max(0, min_cell.x), min(columns, max_cell.x + 1)):
		for y in range(max(0, min_cell.y), min(rows, max_cell.y + 1)):
			if diggable[x][y] == 0 or solids[x][y] == 0:
				continue

			var occupancy := _sample_circle_cell_occupancy(local_center, radius, x, y)
			if occupancy < carve_strength_threshold:
				continue

			solids[x][y] = 0
			diggable[x][y] = 0
			changed = true

	if changed:
		_rebuild_collision_rects()
		_update_visual_surface(1.0 / 60.0)

	return changed


func add_water_blob(world_center: Vector2, radius: float, amount: float) -> void:
	if amount <= 0.0:
		return

	var local_center := to_local(world_center)
	var min_cell := _world_to_cell(local_center - Vector2(radius, radius))
	var max_cell := _world_to_cell(local_center + Vector2(radius, radius))
	var targets: Array[Dictionary] = []
	var total_weight := 0.0

	for x in range(max(0, min_cell.x), min(columns, max_cell.x + 1)):
		for y in range(max(0, min_cell.y), min(rows, max_cell.y + 1)):
			if not _is_open(x, y):
				continue

			var dist := _cell_center(x, y).distance_to(local_center)
			if dist > radius:
				continue

			var weight := maxf(0.05, 1.0 - dist / radius)
			targets.append({
				"x": x,
				"y": y,
				"weight": weight
			})
			total_weight += weight

	if targets.is_empty() or total_weight <= 0.0:
		return

	for target in targets:
		var tx: int = target["x"]
		var ty: int = target["y"]
		var weight: float = target["weight"]
		masses[tx][ty] += amount * weight / total_weight

	queue_impact(world_center, radius * 0.8, amount * 0.08)


func get_goal_fill_ratio() -> float:
	var filled := 0.0
	var counted_cells := 0

	for x in range(goal_rect.position.x, goal_rect.position.x + goal_rect.size.x):
		for y in range(goal_rect.position.y, goal_rect.position.y + goal_rect.size.y):
			if not _is_inside(x, y) or solids[x][y] != 0:
				continue
			filled += clamp(masses[x][y], 0.0, 1.0)
			counted_cells += 1

	if counted_cells == 0:
		return 0.0

	return filled / float(counted_cells)


func sample_circle_fill(world_center: Vector2, radius: float) -> float:
	var local_center := to_local(world_center)
	var min_cell := _world_to_cell(local_center - Vector2(radius, radius))
	var max_cell := _world_to_cell(local_center + Vector2(radius, radius))
	var weighted_mass := 0.0
	var weight_sum := 0.0
	var reach := radius + cell_size * 0.5

	for x in range(max(0, min_cell.x), min(columns, max_cell.x + 1)):
		for y in range(max(0, min_cell.y), min(rows, max_cell.y + 1)):
			if solids[x][y] != 0:
				continue

			var dist := _cell_center(x, y).distance_to(local_center)
			if dist >= reach:
				continue

			var weight := 1.0 - dist / reach
			weighted_mass += clamp(masses[x][y], 0.0, 1.0) * weight
			weight_sum += weight

	if weight_sum <= 0.0:
		return 0.0

	return weighted_mass / weight_sum


func displace_circle(world_center: Vector2, radius: float, intensity: float = 0.08) -> void:
	queue_impact(world_center, radius, intensity)


func _apply_body_constraints() -> void:
	for x in range(columns):
		for y in range(rows):
			if solids[x][y] != 0:
				continue

			var occupancy: float = body_occupancy[x][y]
			if occupancy <= MIN_FLOW:
				continue

			var current_mass: float = float(masses[x][y])
			var capacity := maxf(0.0, 1.0 - occupancy * occupancy_displacement_factor)
			if current_mass <= capacity:
				continue

			var excess := current_mass - capacity
			masses[x][y] = capacity
			if not _place_displaced_mass_on_surface(Vector2i(x, y), excess, x + y):
				masses[x][y] += excess


func _clear_dynamic_inputs() -> void:
	for x in range(body_occupancy.size()):
		for y in range(body_occupancy[x].size()):
			body_occupancy[x][y] = 0.0


func _apply_body_displacement(local_center: Vector2, radius: float, min_cell: Vector2i, max_cell: Vector2i) -> float:
	var removed_mass := 0.0

	for x in range(max(0, min_cell.x), min(columns, max_cell.x + 1)):
		for y in range(max(0, min_cell.y), min(rows, max_cell.y + 1)):
			if solids[x][y] != 0:
				continue

			var occupancy := _sample_circle_cell_occupancy(local_center, radius, x, y)
			if occupancy <= 0.0:
				continue

			var capacity := maxf(0.0, 1.0 - occupancy * occupancy_displacement_factor)
			var current_mass := float(masses[x][y])
			if current_mass <= capacity:
				continue

			var excess := current_mass - capacity
			masses[x][y] = capacity
			removed_mass += excess

	return removed_mass


func _apply_impact_displacement(local_center: Vector2, radius: float, intensity: float, min_cell: Vector2i, max_cell: Vector2i) -> float:
	var removed_mass := 0.0
	if intensity <= 0.0:
		return removed_mass

	for x in range(max(0, min_cell.x), min(columns, max_cell.x + 1)):
		for y in range(max(0, min_cell.y), min(rows, max_cell.y + 1)):
			if solids[x][y] != 0:
				continue

			var dist := _cell_center(x, y).distance_to(local_center)
			if dist >= radius:
				continue

			var falloff := 1.0 - dist / radius
			var removable: float = minf(float(masses[x][y]), falloff * intensity)
			if removable <= 0.0:
				continue

			masses[x][y] -= removable
			removed_mass += removable

	return removed_mass


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, get_playfield_size()), Color(0.1, 0.11, 0.14, 1.0))

	var solid_color := Color(0.26, 0.27, 0.31, 1.0)
	var dirt_color := Color(0.45, 0.33, 0.18, 1.0)
	for x in range(columns):
		for y in range(rows):
			if solids[x][y] != 0:
				var color := solid_color
				if diggable[x][y] != 0:
					color = dirt_color
				draw_rect(Rect2(_cell_origin(x, y), Vector2.ONE * cell_size), color)

	var goal_origin := Vector2(goal_rect.position.x, goal_rect.position.y) * cell_size
	var goal_size := Vector2(goal_rect.size.x, goal_rect.size.y) * cell_size
	draw_rect(Rect2(goal_origin, goal_size), Color(0.16, 0.4, 0.2, 0.3), false, 3.0)
	draw_rect(Rect2(goal_origin, goal_size), Color(0.16, 0.4, 0.2, 0.12), true)

	_draw_water_bodies()
	_draw_surface_wave()


func _draw_water_bodies() -> void:
	for x in range(columns):
		var segments := _get_water_segments_for_column(x)
		for segment_index in range(1, segments.size()):
			var segment = segments[segment_index]
			var segment_top: int = segment[0]
			var segment_bottom: int = segment[1]
			var top_y: float = float(segment[2])
			var water_height: float = float(segment[3])
			var segment_height: float = float(segment_bottom - segment_top + 1) * cell_size
			var fill_ratio: float = clampf(water_height / maxf(segment_height, 1.0), 0.0, 1.0)
			var body_color := Color(0.24 + fill_ratio * 0.06, 0.56 + fill_ratio * 0.07, 0.94, 0.92)
			draw_rect(Rect2(Vector2(float(x) * cell_size, top_y), Vector2(cell_size, water_height)), body_color)


func _draw_surface_wave() -> void:
	var x := 0
	while x < columns:
		if rendered_surface_valid[x] == 0:
			x += 1
			continue

		var start_x := x
		var top_samples: PackedFloat32Array = []
		var bottom_samples: PackedFloat32Array = []

		while x < columns and rendered_surface_valid[x] != 0:
			top_samples.append(rendered_surface_y[x])
			var segment := _get_bottom_water_segment(x)
			bottom_samples.append(float(int(segment[1]) + 1) * cell_size)
			x += 1

		_draw_surface_body(start_x, top_samples, bottom_samples)


func _get_bottom_water_segment(column_x: int) -> Array:
	var segments := _get_water_segments_for_column(column_x)
	if segments.is_empty():
		return []
	return segments[0]


func _get_water_segments_for_column(column_x: int) -> Array:
	var segments := []
	var y := rows - 1
	while y >= 0:
		while y >= 0 and solids[column_x][y] != 0:
			y -= 1

		if y < 0:
			break

		var segment_bottom := y
		var segment_mass := 0.0
		while y >= 0 and solids[column_x][y] == 0:
			segment_mass += clampf(float(masses[column_x][y]), 0.0, MAX_MASS + MAX_COMPRESS)
			y -= 1

		var segment_top := y + 1
		if segment_mass <= MIN_MASS:
			continue

		var segment_cell_count: int = segment_bottom - segment_top + 1
		var max_segment_height: float = float(segment_cell_count) * cell_size
		var water_height: float = minf(segment_mass * cell_size, max_segment_height)
		if water_height <= 1.0:
			continue

		var bottom_y: float = float(segment_bottom + 1) * cell_size
		var top_y: float = bottom_y - water_height
		segments.append([segment_top, segment_bottom, top_y, water_height])

	return segments


func _draw_surface_body(start_x: int, top_samples: PackedFloat32Array, bottom_samples: PackedFloat32Array) -> void:
	if top_samples.is_empty() or bottom_samples.is_empty():
		return

	var smoothed_samples := _smooth_surface_samples(top_samples, 0.0, float(rows) * cell_size)
	var boundary_samples := _build_surface_boundaries(smoothed_samples)
	var fill_color := Color(0.24, 0.57, 0.94, 0.96)
	var foam_color := Color(0.83, 0.93, 1.0, 0.45)
	var fill_polygon := PackedVector2Array()
	var top_polyline := PackedVector2Array()

	for index in range(boundary_samples.size()):
		var px: float = float(start_x + index) * cell_size
		var py: float = boundary_samples[index]
		fill_polygon.append(Vector2(px, py))
		top_polyline.append(Vector2(px, py))

	for index in range(bottom_samples.size() - 1, -1, -1):
		var right_x: float = float(start_x + index + 1) * cell_size
		var left_x: float = float(start_x + index) * cell_size
		var bottom_y: float = bottom_samples[index]
		fill_polygon.append(Vector2(right_x, bottom_y))
		fill_polygon.append(Vector2(left_x, bottom_y))

	if fill_polygon.size() >= 3:
		draw_colored_polygon(fill_polygon, fill_color)

	if top_polyline.size() >= 2:
		draw_polyline(top_polyline, foam_color, 2.0, true)


func _smooth_surface_samples(samples: PackedFloat32Array, min_y: float, max_y: float) -> PackedFloat32Array:
	var smoothed := samples.duplicate()
	for _pass in range(surface_smoothing_passes):
		var next_samples := smoothed.duplicate()
		for index in range(smoothed.size()):
			var sum := smoothed[index] * 2.0
			var weight := 2.0
			if index > 0:
				sum += smoothed[index - 1]
				weight += 1.0
			if index < smoothed.size() - 1:
				sum += smoothed[index + 1]
				weight += 1.0
			next_samples[index] = clampf(sum / weight, min_y, max_y)
		smoothed = next_samples
	return smoothed


func _relax_hydrostatic_basins(delta: float) -> void:
	var visited: Array[PackedByteArray] = _empty_byte_grid()
	var target_masses: Array[PackedFloat32Array] = _empty_grid()
	for x in range(columns):
		for y in range(rows):
			target_masses[x][y] = masses[x][y]

	for x in range(columns):
		for y in range(rows):
			if solids[x][y] != 0 or visited[x][y] != 0:
				continue
			if float(masses[x][y]) <= hydrostatic_mass_threshold:
				continue

			var basin_cells: Array[Vector2i] = []
			var row_buckets := _create_row_buckets()
			var total_mass: float = _collect_relaxation_basin(Vector2i(x, y), visited, basin_cells, row_buckets)
			if total_mass <= MIN_FLOW:
				continue

			for cell in basin_cells:
				target_masses[cell.x][cell.y] = 0.0

			var remaining: float = total_mass
			var fallback_cell := basin_cells[0]
			for row in range(rows - 1, -1, -1):
				var bucket: Array = row_buckets[row]
				for cell_variant in bucket:
					var cell: Vector2i = cell_variant
					fallback_cell = cell
					var capacity: float = _relaxation_cell_capacity(cell.x, cell.y)
					if capacity <= MIN_FLOW:
						continue

					var assigned: float = minf(capacity, remaining)
					target_masses[cell.x][cell.y] = assigned
					remaining -= assigned
					if remaining <= MIN_FLOW:
						break
				if remaining <= MIN_FLOW:
					break

			if remaining > MIN_FLOW:
				target_masses[fallback_cell.x][fallback_cell.y] += remaining

	var relax_alpha: float = clampf(1.0 - exp(-hydrostatic_relaxation_speed * delta), 0.0, 1.0)
	for x in range(columns):
		for y in range(rows):
			if solids[x][y] != 0:
				continue
			masses[x][y] = lerpf(float(masses[x][y]), float(target_masses[x][y]), relax_alpha)


func _collect_relaxation_basin(start: Vector2i, visited: Array[PackedByteArray], basin_cells: Array[Vector2i], row_buckets: Array) -> float:
	var basin_queue: Array[Vector2i] = []
	basis_queue_append_unique(basin_queue, visited, start)
	var basin_head: int = 0
	var total_mass := 0.0
	while basin_head < basin_queue.size():
		var basin_cell: Vector2i = basin_queue[basin_head]
		basin_head += 1
		basin_cells.append(basin_cell)
		var row_bucket: Array = row_buckets[basin_cell.y]
		row_bucket.append(basin_cell)
		total_mass += float(masses[basin_cell.x][basin_cell.y])

		for neighbor in _neighbor_cells(basin_cell):
			if not _is_open(neighbor.x, neighbor.y):
				continue
			if visited[neighbor.x][neighbor.y] != 0:
				continue
			if float(masses[neighbor.x][neighbor.y]) <= hydrostatic_mass_threshold:
				continue
			basis_queue_append_unique(basin_queue, visited, neighbor)

	return total_mass


func basis_queue_append_unique(queue: Array[Vector2i], visited: Array[PackedByteArray], cell: Vector2i) -> void:
	if visited[cell.x][cell.y] != 0:
		return
	visited[cell.x][cell.y] = 1
	queue.append(cell)


func _relaxation_cell_capacity(x: int, y: int) -> float:
	return maxf(0.0, MAX_MASS - body_occupancy[x][y] * occupancy_displacement_factor)


func _neighbor_cells(cell: Vector2i) -> Array[Vector2i]:
	var neighbors: Array[Vector2i] = []
	neighbors.append(Vector2i(cell.x + 1, cell.y))
	neighbors.append(Vector2i(cell.x - 1, cell.y))
	neighbors.append(Vector2i(cell.x, cell.y + 1))
	neighbors.append(Vector2i(cell.x, cell.y - 1))
	return neighbors


func _displacement_neighbors(cell: Vector2i, spread_seed: int) -> Array[Vector2i]:
	var neighbors: Array[Vector2i] = []
	neighbors.append(Vector2i(cell.x, cell.y - 1))
	if spread_seed % 2 == 0:
		neighbors.append(Vector2i(cell.x - 1, cell.y))
		neighbors.append(Vector2i(cell.x + 1, cell.y))
	else:
		neighbors.append(Vector2i(cell.x + 1, cell.y))
		neighbors.append(Vector2i(cell.x - 1, cell.y))
	neighbors.append(Vector2i(cell.x, cell.y + 1))
	return neighbors


func _create_row_buckets() -> Array:
	var buckets: Array = []
	buckets.resize(rows)
	for row in range(rows):
		buckets[row] = []
	return buckets


func _empty_byte_grid() -> Array[PackedByteArray]:
	var grid: Array[PackedByteArray] = []
	for x in range(columns):
		var column := PackedByteArray()
		column.resize(rows)
		for y in range(rows):
			column[y] = 0
		grid.append(column)
	return grid


func _build_surface_boundaries(samples: PackedFloat32Array) -> PackedFloat32Array:
	var boundaries := PackedFloat32Array()
	if samples.is_empty():
		return boundaries

	boundaries.resize(samples.size() + 1)
	boundaries[0] = samples[0]
	for index in range(1, samples.size()):
		boundaries[index] = (samples[index - 1] + samples[index]) * 0.5
	boundaries[samples.size()] = samples[samples.size() - 1]
	return boundaries


func _simulate_step() -> void:
	var delta: Array[PackedFloat32Array] = _empty_grid()

	for y in range(rows - 1, -1, -1):
		if scan_left_to_right:
			for x in range(columns):
				_simulate_cell(x, y, delta)
		else:
			for x in range(columns - 1, -1, -1):
				_simulate_cell(x, y, delta)

	scan_left_to_right = not scan_left_to_right

	for x in range(columns):
		for y in range(rows):
			if solids[x][y] != 0:
				masses[x][y] = 0.0
				continue

			masses[x][y] = max(0.0, masses[x][y] + delta[x][y])
			if masses[x][y] < MIN_MASS:
				masses[x][y] = 0.0


func _simulate_cell(x: int, y: int, delta: Array) -> void:
	if solids[x][y] != 0:
		return

	var remaining: float = float(masses[x][y])
	if remaining <= MIN_MASS:
		return

	if _is_open(x, y + 1):
		var below_mass: float = float(masses[x][y + 1])
		var flow_down: float = _stable_mass(remaining + below_mass) - below_mass
		if flow_down > MIN_FLOW:
			flow_down = min(flow_down, min(MAX_FLOW, remaining))
			delta[x][y] -= flow_down
			delta[x][y + 1] += flow_down
			remaining -= flow_down

	if remaining <= MIN_MASS:
		return

	var directions: Array[int]
	var current_pressure: float = _pressure_head(x, y)
	if scan_left_to_right:
		directions = [-1, 1]
	else:
		directions = [1, -1]
	for direction in directions:
		var nx: int = x + direction
		if not _is_open(nx, y):
			continue

		var neighbor_pressure: float = _pressure_head(nx, y)
		var pressure_delta: float = current_pressure - neighbor_pressure
		if pressure_delta <= MIN_FLOW:
			continue

		var flow_side: float = pressure_delta * lateral_flow_factor
		if flow_side <= MIN_FLOW:
			continue

		flow_side = minf(flow_side, minf(MAX_FLOW * 0.5, remaining))
		delta[x][y] -= flow_side
		delta[nx][y] += flow_side
		remaining -= flow_side
		current_pressure = maxf(0.0, current_pressure - flow_side)

		if remaining <= MIN_MASS:
			return

	if _is_open(x, y - 1):
		var above_mass: float = float(masses[x][y - 1])
		var total_mass: float = remaining + above_mass
		var flow_up: float = total_mass - _stable_mass(total_mass)
		if flow_up > MIN_FLOW:
			flow_up = min(flow_up * upward_flow_factor, remaining)
			delta[x][y] -= flow_up
			delta[x][y - 1] += flow_up


func _init_arrays() -> void:
	masses.clear()
	solids.clear()
	diggable.clear()
	body_occupancy.clear()
	collision_rects.clear()

	for x in range(columns):
		var mass_column := PackedFloat32Array()
		var solid_column := PackedByteArray()
		var diggable_column := PackedByteArray()
		var occupancy_column := PackedFloat32Array()
		mass_column.resize(rows)
		solid_column.resize(rows)
		diggable_column.resize(rows)
		occupancy_column.resize(rows)

		for y in range(rows):
			mass_column[y] = 0.0
			solid_column[y] = 0
			diggable_column[y] = 0
			occupancy_column[y] = 0.0

		masses.append(mass_column)
		solids.append(solid_column)
		diggable.append(diggable_column)
		body_occupancy.append(occupancy_column)


func _build_level() -> void:
	_add_solid_rect(0, rows - 1, columns, 1, false)
	_add_solid_rect(0, 0, 1, rows, false)
	_add_solid_rect(columns - 1, 0, 1, rows, false)
	_add_solid_rect(16, 11, 1, rows - 11, false)
	_add_solid_rect(17, 20, 8, 3, true)
	_add_solid_rect(25, 21, 5, 2, true)

	for x in range(1, 16):
		for y in range(12, rows - 1):
			if solids[x][y] == 0:
				masses[x][y] = 1.0


func _add_solid_rect(start_x: int, start_y: int, width: int, height: int, is_diggable: bool) -> void:
	for x in range(start_x, start_x + width):
		for y in range(start_y, start_y + height):
			if _is_inside(x, y):
				solids[x][y] = 1
				if is_diggable:
					diggable[x][y] = 1
				masses[x][y] = 0.0


func _place_splash_mass(center_cell: Vector2i, amount: float, splash_seed: int) -> bool:
	var offsets := [
		Vector2i(0, -2),
		Vector2i(-1, -1),
		Vector2i(1, -1),
		Vector2i(0, -1),
		Vector2i(-2, -1),
		Vector2i(2, -1),
		Vector2i(-1, 0),
		Vector2i(1, 0),
		Vector2i(-3, 0),
		Vector2i(3, 0),
		Vector2i(0, 1)
	]

	for index in range(offsets.size()):
		var offset: Vector2i = offsets[(index + splash_seed) % offsets.size()]
		var tx := center_cell.x + offset.x
		var ty := center_cell.y + offset.y
		if not _is_open(tx, ty):
			continue
		masses[tx][ty] += amount
		return true

	for radius in range(1, 6):
		for dy in range(-radius, radius + 1):
			for dx in range(-radius, radius + 1):
				var tx := center_cell.x + dx
				var ty := center_cell.y + dy
				if not _is_open(tx, ty):
					continue
				masses[tx][ty] += amount
				return true

	return false


func _place_displaced_mass(center_cell: Vector2i, amount: float, spread_seed: int) -> bool:
	var visited: Array[PackedByteArray] = _empty_byte_grid()
	var queue: Array[Vector2i] = []
	var head: int = 0
	var min_y: int = center_cell.y - displacement_search_radius
	if min_y < 0:
		min_y = 0

	queue.append(center_cell)
	visited[center_cell.x][center_cell.y] = 1

	while head < queue.size():
		var cell: Vector2i = queue[head]
		head += 1

		if cell != center_cell and _is_open(cell.x, cell.y):
			masses[cell.x][cell.y] += amount
			return true

		for neighbor in _displacement_neighbors(cell, spread_seed):
			if not _is_inside(neighbor.x, neighbor.y):
				continue
			if neighbor.y < min_y:
				continue
			if abs(neighbor.x - center_cell.x) > displacement_search_radius:
				continue
			if abs(neighbor.y - center_cell.y) > displacement_search_radius + 1:
				continue
			if visited[neighbor.x][neighbor.y] != 0:
				continue
			if not _is_open(neighbor.x, neighbor.y):
				continue

			visited[neighbor.x][neighbor.y] = 1
			queue.append(neighbor)

	return false


func _place_displaced_mass_on_surface(center_cell: Vector2i, amount: float, spread_seed: int) -> bool:
	if not _is_open(center_cell.x, center_cell.y):
		return false
	if float(masses[center_cell.x][center_cell.y]) <= hydrostatic_mass_threshold:
		return false

	var visited: Array[PackedByteArray] = _empty_byte_grid()
	var queue: Array[Vector2i] = []
	var head: int = 0
	var top_by_column := PackedInt32Array()
	top_by_column.resize(columns)
	for x in range(columns):
		top_by_column[x] = rows

	queue.append(center_cell)
	visited[center_cell.x][center_cell.y] = 1

	while head < queue.size():
		var cell: Vector2i = queue[head]
		head += 1

		if cell.y < top_by_column[cell.x]:
			top_by_column[cell.x] = cell.y

		for neighbor in _neighbor_cells(cell):
			if not _is_open(neighbor.x, neighbor.y):
				continue
			if visited[neighbor.x][neighbor.y] != 0:
				continue
			if float(masses[neighbor.x][neighbor.y]) <= hydrostatic_mass_threshold:
				continue

			visited[neighbor.x][neighbor.y] = 1
			queue.append(neighbor)

	var candidates: Array[Vector2i] = []
	for distance in range(columns + rows):
		var found_any := false
		for offset in _offset_order(distance, spread_seed):
			var target_x := center_cell.x + offset
			if target_x < 0 or target_x >= columns:
				continue

			var target_y := top_by_column[target_x]
			if target_y >= rows:
				continue

			found_any = true
			candidates.append(Vector2i(target_x, target_y))

		if found_any and candidates.size() >= 1:
			continue

	if candidates.is_empty():
		return false

	var split_amount := amount / float(candidates.size())
	for candidate in candidates:
		masses[candidate.x][candidate.y] += split_amount

	return true


func _stable_mass(total_mass: float) -> float:
	if total_mass <= MAX_MASS:
		return MAX_MASS
	if total_mass < 2.0 * MAX_MASS + MAX_COMPRESS:
		return (MAX_MASS * MAX_MASS + total_mass * MAX_COMPRESS) / (MAX_MASS + MAX_COMPRESS)
	return (total_mass + MAX_COMPRESS) * 0.5


func _empty_grid() -> Array[PackedFloat32Array]:
	var grid: Array[PackedFloat32Array] = []
	for x in range(columns):
		var column := PackedFloat32Array()
		column.resize(rows)
		for y in range(rows):
			column[y] = 0.0
		grid.append(column)
	return grid


func _rebuild_collision_rects() -> void:
	collision_rects.clear()
	for y in range(rows):
		var run_start := -1
		for x in range(columns + 1):
			var solid := x < columns and solids[x][y] != 0
			if solid and run_start == -1:
				run_start = x
			elif not solid and run_start != -1:
				collision_rects.append(Rect2(run_start * cell_size, y * cell_size, (x - run_start) * cell_size, cell_size))
				run_start = -1


func _initialize_visual_surface() -> void:
	rendered_surface_y.resize(columns)
	rendered_surface_target_y.resize(columns)
	rendered_surface_velocity.resize(columns)
	rendered_surface_min_y.resize(columns)
	rendered_surface_max_y.resize(columns)
	rendered_surface_valid.resize(columns)
	for x in range(columns):
		var segment := _get_bottom_water_segment(x)
		if segment.is_empty():
			var bottom_y := float(rows) * cell_size
			rendered_surface_y[x] = bottom_y
			rendered_surface_target_y[x] = bottom_y
			rendered_surface_velocity[x] = 0.0
			rendered_surface_min_y[x] = 0.0
			rendered_surface_max_y[x] = bottom_y
			rendered_surface_valid[x] = 0
		else:
			var target_y: float = float(segment[2])
			rendered_surface_y[x] = target_y
			rendered_surface_target_y[x] = target_y
			rendered_surface_velocity[x] = 0.0
			rendered_surface_min_y[x] = float(segment[0]) * cell_size
			rendered_surface_max_y[x] = float(segment[1] + 1) * cell_size
			rendered_surface_valid[x] = 1


func _update_visual_surface(delta: float) -> void:
	if rendered_surface_y.size() != columns or rendered_surface_target_y.size() != columns or rendered_surface_velocity.size() != columns or rendered_surface_valid.size() != columns:
		_initialize_visual_surface()

	_update_surface_targets()
	_apply_surface_impacts()

	var substeps: int = wave_substeps
	if substeps < 1:
		substeps = 1
	var sub_delta: float = delta / float(substeps)
	var damping_factor: float = exp(-wave_damping * sub_delta)
	for _step in range(substeps):
		var next_velocity := rendered_surface_velocity.duplicate()
		for x in range(columns):
			if rendered_surface_valid[x] == 0:
				next_velocity[x] = 0.0
				continue

			var acceleration: float = (rendered_surface_target_y[x] - rendered_surface_y[x]) * wave_tension
			if _surface_columns_connected(x, x - 1):
				acceleration += (rendered_surface_y[x - 1] - rendered_surface_y[x]) * wave_spread
			if _surface_columns_connected(x, x + 1):
				acceleration += (rendered_surface_y[x + 1] - rendered_surface_y[x]) * wave_spread

			next_velocity[x] = (rendered_surface_velocity[x] + acceleration * sub_delta) * damping_factor

		for x in range(columns):
			if rendered_surface_valid[x] == 0:
				rendered_surface_velocity[x] = 0.0
				continue

			rendered_surface_velocity[x] = next_velocity[x]
			rendered_surface_y[x] += rendered_surface_velocity[x] * sub_delta
			rendered_surface_y[x] = clampf(rendered_surface_y[x], rendered_surface_min_y[x], rendered_surface_max_y[x])
			if rendered_surface_y[x] == rendered_surface_min_y[x] or rendered_surface_y[x] == rendered_surface_max_y[x]:
				rendered_surface_velocity[x] *= 0.35


func _update_surface_targets() -> void:
	for x in range(columns):
		var segment := _get_bottom_water_segment(x)
		if segment.is_empty():
			var bottom_y := float(rows) * cell_size
			rendered_surface_target_y[x] = bottom_y
			rendered_surface_y[x] = lerpf(rendered_surface_y[x], bottom_y, clampf(surface_follow_speed * 0.08, 0.0, 1.0))
			rendered_surface_min_y[x] = 0.0
			rendered_surface_max_y[x] = bottom_y
			rendered_surface_valid[x] = 0
			continue

		var target_y: float = float(segment[2])
		var min_y: float = float(segment[0]) * cell_size
		var max_y: float = float(segment[1] + 1) * cell_size
		if rendered_surface_valid[x] == 0:
			rendered_surface_y[x] = target_y
			rendered_surface_velocity[x] = 0.0
			rendered_surface_valid[x] = 1

		rendered_surface_target_y[x] = target_y
		rendered_surface_min_y[x] = min_y
		rendered_surface_max_y[x] = max_y


func _apply_surface_impacts() -> void:
	for impact in queued_surface_impacts:
		var local_center: Vector2 = to_local(impact["center"])
		var radius: float = float(impact["radius"])
		var intensity: float = float(impact["intensity"])
		var reach: float = maxf(radius * 2.5, cell_size)
		for x in range(columns):
			if rendered_surface_valid[x] == 0:
				continue

			var dist_x: float = absf(_cell_center(x, 0).x - local_center.x)
			if dist_x > reach:
				continue

			var falloff: float = 1.0 - dist_x / reach
			rendered_surface_velocity[x] += intensity * wave_impact_strength * falloff

	queued_surface_impacts.clear()


func _surface_columns_connected(a: int, b: int) -> bool:
	if a < 0 or a >= columns or b < 0 or b >= columns:
		return false
	return rendered_surface_valid[a] != 0 and rendered_surface_valid[b] != 0


func _get_rendered_surface_top(column_x: int, fallback_top: float) -> float:
	if column_x < 0 or column_x >= rendered_surface_y.size():
		return fallback_top
	return rendered_surface_y[column_x]


func _sample_circle_cell_occupancy(local_center: Vector2, radius: float, x: int, y: int) -> float:
	var cell_origin := _cell_origin(x, y)
	var inside_samples := 0.0
	var total_samples := 0.0
	var radius_squared := radius * radius

	for sample_x in range(3):
		for sample_y in range(3):
			var normalized_x := (float(sample_x) + 0.5) / 3.0
			var normalized_y := (float(sample_y) + 0.5) / 3.0
			var sample_position := cell_origin + Vector2(normalized_x * cell_size, normalized_y * cell_size)
			if sample_position.distance_squared_to(local_center) <= radius_squared:
				inside_samples += 1.0
			total_samples += 1.0

	if total_samples <= 0.0:
		return 0.0

	return inside_samples / total_samples


func _pressure_head(x: int, y: int) -> float:
	if not _is_open(x, y):
		return 0.0

	var pressure: float = float(masses[x][y])
	var scan_y: int = y - 1
	while scan_y >= 0 and _is_open(x, scan_y) and float(masses[x][scan_y]) > MIN_MASS:
		pressure += float(masses[x][scan_y])
		scan_y -= 1

	return pressure


func _offset_with_seed(distance: int, spread_seed: int) -> int:
	if distance == 0:
		return 0
	if spread_seed % 2 == 0:
		return distance
	return -distance


func _offset_order(distance: int, spread_seed: int) -> Array[int]:
	if distance == 0:
		return [0]
	if spread_seed % 2 == 0:
		return [-distance, distance]
	return [distance, -distance]


func _has_horizontal_barrier(from_x: int, to_x: int, reference_y: int) -> bool:
	if from_x == to_x:
		return false

	var step := 1 if to_x > from_x else -1
	var scan_x := from_x + step
	var y := clampi(reference_y, 0, rows - 1)
	while scan_x != to_x:
		if solids[scan_x][y] != 0:
			return true
		scan_x += step

	return false


func _world_to_cell(local_position: Vector2) -> Vector2i:
	return Vector2i(floori(local_position.x / cell_size), floori(local_position.y / cell_size))


func _cell_center(x: int, y: int) -> Vector2:
	return Vector2((x + 0.5) * cell_size, (y + 0.5) * cell_size)


func _cell_origin(x: int, y: int) -> Vector2:
	return Vector2(x * cell_size, y * cell_size)


func _is_inside(x: int, y: int) -> bool:
	return x >= 0 and x < columns and y >= 0 and y < rows


func _is_open(x: int, y: int) -> bool:
	return _is_inside(x, y) and solids[x][y] == 0