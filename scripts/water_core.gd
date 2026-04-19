extends Node2D

class DropletData:
	var position: Vector2
	var velocity: Vector2
	var volume: float
	var radius: float

	func _init(p_position: Vector2, p_velocity: Vector2, p_volume: float, p_radius: float) -> void:
		position = p_position
		velocity = p_velocity
		volume = p_volume
		radius = p_radius


class SpillVisualData:
	var start: Vector2
	var end: Vector2
	var width: float
	var life: float

	func _init(p_start: Vector2, p_end: Vector2, p_width: float, p_life: float) -> void:
		start = p_start
		end = p_end
		width = p_width
		life = p_life


@export var columns := 40
@export var rows := 24
@export var cell_size := 24.0
@export var goal_required_ratio := 0.55
@export var simulation_steps := 6
@export var flow_speed := 9.5
@export var max_transfer_per_second := 18.0
@export var body_displacement_factor := 0.8
@export var surface_smoothing_passes := 3
@export var droplet_gravity := 1500.0
@export var droplet_horizontal_damping := 0.985
@export var spill_droplet_threshold := 0.025
@export var spill_visual_lifetime := 0.12
@export var spill_visual_width := 6.0
@export var render_volume_threshold := 0.005
@export var min_render_water_height := 2.0
@export var spill_lip_cover := 8.0
@export var carve_strength_threshold := 0.3

const MIN_VOLUME := 0.0001
const SURFACE_EPSILON := 0.01
const WATER_FILL_COLOR := Color(0.24, 0.57, 0.94, 0.96)
const WATER_FOAM_COLOR := Color(0.84, 0.93, 1.0, 0.5)
const WATER_DROP_COLOR := Color(0.7, 0.88, 1.0, 0.9)

var solids: Array[PackedByteArray] = []
var diggable: Array[PackedByteArray] = []
var collision_rects: Array[Rect2] = []
var goal_rect := Rect2i(31, 19, 7, 4)

var column_floor_row := PackedInt32Array()
var column_capacity := PackedFloat32Array()
var column_bottom_y := PackedFloat32Array()
var water_volume := PackedFloat32Array()
var body_displacement := PackedFloat32Array()
var edge_spill_flow := PackedFloat32Array()
var edge_spill_source_y := PackedFloat32Array()
var edge_spill_target_y := PackedFloat32Array()
var edge_spill_direction := PackedInt32Array()
var droplets: Array[DropletData] = []
var spill_visuals: Array[SpillVisualData] = []


func _ready() -> void:
	reset_level()


func _physics_process(delta: float) -> void:
	_clear_edge_spills()
	_simulate_droplets(delta)
	_simulate_columns(delta)
	_update_spill_visuals(delta)
	_clear_body_displacement()
	queue_redraw()


func reset_level() -> void:
	_init_arrays()
	_build_level()
	_rebuild_collision_rects()
	_rebuild_column_profiles()
	_seed_initial_water()
	droplets.clear()
	spill_visuals.clear()
	queue_redraw()


func get_playfield_size() -> Vector2:
	return Vector2(columns * cell_size, rows * cell_size)


func get_collision_rects() -> Array[Rect2]:
	return collision_rects.duplicate()


func get_goal_fill_ratio() -> float:
	var filled := 0.0
	var counted_cells := 0

	for x in range(goal_rect.position.x, goal_rect.position.x + goal_rect.size.x):
		if x < 0 or x >= columns or not _column_is_open(x):
			continue

		var surface_y := _surface_y_from_volume(x, water_volume[x])
		for y in range(goal_rect.position.y, goal_rect.position.y + goal_rect.size.y):
			if y < 0 or y >= rows:
				continue
			if float(y) >= column_capacity[x]:
				continue

			var cell_top := float(y) * cell_size
			var cell_bottom := float(y + 1) * cell_size
			var fill := clampf((cell_bottom - maxf(cell_top, surface_y)) / cell_size, 0.0, 1.0)
			filled += fill
			counted_cells += 1

	if counted_cells == 0:
		return 0.0

	return filled / float(counted_cells)


func sample_circle_fill(world_center: Vector2, radius: float) -> float:
	var local_center := to_local(world_center)
	var min_cell := _world_to_cell(local_center - Vector2(radius, radius))
	var max_cell := _world_to_cell(local_center + Vector2(radius, radius))
	var wet_weight := 0.0
	var total_weight := 0.0

	for x in range(max(0, min_cell.x), min(columns, max_cell.x + 1)):
		if not _column_is_open(x):
			continue

		var surface_y := _surface_y_from_volume(x, water_volume[x])
		for y in range(max(0, min_cell.y), min(rows, max_cell.y + 1)):
			if solids[x][y] != 0:
				continue

			var occupancy := _sample_circle_cell_occupancy(local_center, radius, x, y)
			if occupancy <= 0.0:
				continue

			var cell_top := float(y) * cell_size
			var cell_bottom := float(y + 1) * cell_size
			var wet_fraction := clampf((cell_bottom - maxf(cell_top, surface_y)) / cell_size, 0.0, 1.0)
			total_weight += occupancy
			wet_weight += occupancy * wet_fraction

	if total_weight <= 0.0:
		return 0.0

	return wet_weight / total_weight


func register_circle_body(world_center: Vector2, radius: float) -> void:
	var local_center := to_local(world_center)
	var min_cell := _world_to_cell(local_center - Vector2(radius, radius))
	var max_cell := _world_to_cell(local_center + Vector2(radius, radius))

	for x in range(max(0, min_cell.x), min(columns, max_cell.x + 1)):
		if not _column_is_open(x):
			continue

		var surface_y := _surface_y_from_volume(x, water_volume[x])
		var displaced := 0.0
		for y in range(max(0, min_cell.y), min(rows, max_cell.y + 1)):
			if solids[x][y] != 0:
				continue

			var occupancy := _sample_circle_cell_occupancy(local_center, radius, x, y)
			if occupancy <= 0.0:
				continue

			var cell_top := float(y) * cell_size
			var cell_bottom := float(y + 1) * cell_size
			var submerged_fraction := clampf((cell_bottom - maxf(cell_top, surface_y)) / cell_size, 0.0, 1.0)
			displaced += occupancy * submerged_fraction

		body_displacement[x] = minf(column_capacity[x], body_displacement[x] + displaced)


func queue_impact(world_center: Vector2, radius: float, intensity: float) -> void:
	if intensity <= 0.0:
		return

	var local_center := to_local(world_center)
	var splash_count := int(ceil(intensity * 6.0))
	if splash_count < 2:
		splash_count = 2

	for index in range(splash_count):
		var ratio := 0.0
		if splash_count > 1:
			ratio = float(index) / float(splash_count - 1) - 0.5
		var offset_x := ratio * radius * 1.25
		var speed_x := ratio * 160.0
		var speed_y := -140.0 - intensity * 120.0
		_spawn_droplet(local_center + Vector2(offset_x, -radius * 0.1), Vector2(speed_x, speed_y), 0.0)


func displace_circle(world_center: Vector2, radius: float, intensity: float = 0.08) -> void:
	queue_impact(world_center, radius, intensity)


func add_water_blob(world_center: Vector2, radius: float, amount: float) -> void:
	if amount <= 0.0:
		return

	var local_center := to_local(world_center)
	var drop_count := int(ceil(amount * 6.0))
	if drop_count < 1:
		drop_count = 1
	var per_drop := amount / float(drop_count)

	for index in range(drop_count):
		var ratio := 0.0
		if drop_count > 1:
			ratio = float(index) / float(drop_count - 1) - 0.5
		var offset_x := ratio * radius * 0.7
		var speed_x := ratio * 60.0
		_spawn_droplet(local_center + Vector2(offset_x, -radius * 0.2), Vector2(speed_x, 0.0), per_drop)


func carve_circle(world_center: Vector2, radius: float) -> bool:
	var local_center: Vector2 = to_local(world_center)
	var min_x: int = floori((local_center.x - radius) / cell_size)
	if min_x < 0:
		min_x = 0
	var max_x: int = floori((local_center.x + radius) / cell_size)
	if max_x > columns - 1:
		max_x = columns - 1
	var changed: bool = false

	for x in range(min_x, max_x + 1):
		var top_diggable: int = _top_diggable_cell(x)
		if top_diggable == -1:
			continue

		var y: int = top_diggable
		var started: bool = false
		while y < rows and diggable[x][y] != 0:
			var within: bool = _cell_center(x, y).distance_to(local_center) <= radius
			if within:
				solids[x][y] = 0
				diggable[x][y] = 0
				changed = true
				started = true
			elif started:
				break
			y += 1

	if changed:
		_rebuild_collision_rects()
		_rebuild_column_profiles()
		for x in range(columns):
			water_volume[x] = minf(water_volume[x], column_capacity[x])
		queue_redraw()

	return changed


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, get_playfield_size()), Color(0.1, 0.11, 0.14, 1.0))
	_draw_terrain()
	_draw_goal()
	_draw_water()
	_draw_edge_spills()
	_draw_spill_visuals()
	_draw_droplets()


func _draw_terrain() -> void:
	var solid_color := Color(0.26, 0.27, 0.31, 1.0)
	var dirt_color := Color(0.45, 0.33, 0.18, 1.0)

	for x in range(columns):
		for y in range(rows):
			if solids[x][y] == 0:
				continue
			var color := solid_color
			if diggable[x][y] != 0:
				color = dirt_color
			draw_rect(Rect2(_cell_origin(x, y), Vector2.ONE * cell_size), color)


func _draw_goal() -> void:
	var goal_origin := Vector2(goal_rect.position.x, goal_rect.position.y) * cell_size
	var goal_size := Vector2(goal_rect.size.x, goal_rect.size.y) * cell_size
	draw_rect(Rect2(goal_origin, goal_size), Color(0.16, 0.4, 0.2, 0.3), false, 3.0)
	draw_rect(Rect2(goal_origin, goal_size), Color(0.16, 0.4, 0.2, 0.12), true)


func _draw_water() -> void:
	var x := 0
	while x < columns:
		if not _should_render_column(x):
			x += 1
			continue

		var start_x := x
		var top_samples := PackedFloat32Array()
		var bottom_samples := PackedFloat32Array()
		while x < columns and _should_render_column(x):
			if x > start_x and not _should_continue_surface_region(x - 1, x):
				break
			top_samples.append(_effective_surface_y(x))
			bottom_samples.append(column_bottom_y[x])
			x += 1

		_draw_water_region(start_x, top_samples, bottom_samples)


func _draw_water_region(start_x: int, top_samples: PackedFloat32Array, bottom_samples: PackedFloat32Array) -> void:
	if top_samples.is_empty() or bottom_samples.is_empty():
		return

	var smoothed := _smooth_surface_samples(top_samples)
	var top_boundaries := _build_surface_boundaries(smoothed)
	var polyline := PackedVector2Array()

	for index in range(top_boundaries.size()):
		var px := float(start_x + index) * cell_size
		var py := top_boundaries[index]
		polyline.append(Vector2(px, py))

	for index in range(bottom_samples.size()):
		var left_x := float(start_x + index) * cell_size
		var right_x := float(start_x + index + 1) * cell_size
		var top_left := top_boundaries[index]
		var top_right := top_boundaries[index + 1]
		var bottom_y := bottom_samples[index]
		if bottom_y - top_left < min_render_water_height:
			top_left = bottom_y - min_render_water_height
		if bottom_y - top_right < min_render_water_height:
			top_right = bottom_y - min_render_water_height
		if bottom_y - minf(top_left, top_right) <= 0.05:
			continue

		var quad := PackedVector2Array([
			Vector2(left_x, top_left),
			Vector2(right_x, top_right),
			Vector2(right_x, bottom_y),
			Vector2(left_x, bottom_y)
		])
		draw_colored_polygon(quad, WATER_FILL_COLOR)

	if polyline.size() >= 2:
		draw_polyline(polyline, WATER_FOAM_COLOR, 2.0, true)


func _draw_droplets() -> void:
	for droplet in droplets:
		draw_circle(droplet.position, droplet.radius, WATER_DROP_COLOR)


func _draw_spill_visuals() -> void:
	for spill in spill_visuals:
		var alpha := clampf(spill.life / maxf(spill_visual_lifetime, 0.001), 0.0, 1.0)
		var color := Color(WATER_DROP_COLOR.r, WATER_DROP_COLOR.g, WATER_DROP_COLOR.b, 0.55 * alpha)
		draw_line(spill.start, spill.end, color, spill.width * alpha, true)
	
func _draw_edge_spills() -> void:
	for edge in range(edge_spill_flow.size()):
		var flow := edge_spill_flow[edge]
		if flow <= MIN_VOLUME:
			continue

		var direction: int = edge_spill_direction[edge]
		var lip_x: float = float(edge + 1) * cell_size
		var start_y: float = edge_spill_source_y[edge]
		var end_y: float = edge_spill_target_y[edge]
		var width: float = spill_visual_width + minf(8.0, flow * 40.0)
		var alpha := clampf(flow * 10.0, 0.18, 0.85)
		var color := Color(WATER_DROP_COLOR.r, WATER_DROP_COLOR.g, WATER_DROP_COLOR.b, alpha)
		var end_x: float = lip_x + float(direction) * maxf(3.0, width * 0.18)
		var start := Vector2(lip_x, start_y)
		var end := Vector2(end_x, end_y)
		var direction_vector := end - start
		if direction_vector.length_squared() < 0.0001:
			direction_vector = Vector2(0.0, 1.0)
		else:
			direction_vector = direction_vector.normalized()
		var normal := Vector2(-direction_vector.y, direction_vector.x) * width * 0.5
		var sheet := PackedVector2Array([
			start - normal,
			start + normal,
			end + normal * 0.7,
			end - normal * 0.7
		])
		draw_colored_polygon(sheet, color)
		draw_line(start, end, WATER_FOAM_COLOR, maxf(1.5, width * 0.18), true)

		var cap_height: float = maxf(1.4, width * 0.22)
		var cap_origin_x: float = lip_x - spill_lip_cover if direction > 0 else lip_x
		var cap_rect := Rect2(Vector2(cap_origin_x, start_y), Vector2(spill_lip_cover, cap_height))
		draw_rect(cap_rect, color)
		draw_line(Vector2(cap_rect.position.x, start_y), Vector2(cap_rect.position.x + cap_rect.size.x, start_y), WATER_FOAM_COLOR, 1.4)


func _simulate_columns(delta: float) -> void:
	var substeps := simulation_steps
	if substeps < 1:
		substeps = 1
	var sub_delta := delta / float(substeps)

	for _step in range(substeps):
		var delta_volume := PackedFloat32Array()
		delta_volume.resize(columns)
		for x in range(columns - 1):
			if not _column_is_open(x) or not _column_is_open(x + 1):
				continue

			var connection_y := _connection_height_y(x, x + 1)
			var surface_x := _effective_surface_y(x)
			var surface_next := _effective_surface_y(x + 1)
			if surface_x >= connection_y and surface_next >= connection_y:
				continue

			var difference := surface_next - surface_x
			if absf(difference) <= SURFACE_EPSILON:
				continue

			var transfer := absf(difference) / cell_size * flow_speed * sub_delta
			var max_transfer := max_transfer_per_second * sub_delta
			if transfer > max_transfer:
				transfer = max_transfer

			if difference > 0.0:
				transfer = minf(transfer, water_volume[x])
				if _has_step_down(x, x + 1):
					_register_edge_spill(x, x + 1, transfer)
				if _should_spill_as_droplet(x, x + 1, transfer):
					delta_volume[x] -= transfer
					_spawn_spill_droplet(x, x + 1, transfer)
				else:
					delta_volume[x] -= transfer
					delta_volume[x + 1] += transfer
			else:
				transfer = minf(transfer, water_volume[x + 1])
				if _has_step_down(x + 1, x):
					_register_edge_spill(x + 1, x, transfer)
				if _should_spill_as_droplet(x + 1, x, transfer):
					delta_volume[x] += transfer
					delta_volume[x + 1] -= transfer
					_spawn_spill_droplet(x + 1, x, transfer)
				else:
					delta_volume[x] += transfer
					delta_volume[x + 1] -= transfer

		for x in range(columns):
			water_volume[x] += delta_volume[x]
			if water_volume[x] < MIN_VOLUME:
				water_volume[x] = 0.0
			if water_volume[x] > column_capacity[x]:
				var excess := water_volume[x] - column_capacity[x]
				water_volume[x] = column_capacity[x]
				_spill_excess_as_droplets(x, excess)


func _simulate_droplets(delta: float) -> void:
	var playfield_height := float(rows) * cell_size
	for index in range(droplets.size() - 1, -1, -1):
		var droplet := droplets[index]
		droplet.velocity.y += droplet_gravity * delta
		droplet.velocity.x *= pow(droplet_horizontal_damping, delta * 60.0)
		droplet.position += droplet.velocity * delta

		if droplet.position.y - droplet.radius > playfield_height + cell_size:
			droplets.remove_at(index)
			continue

		var receiver := _find_receiver_column(droplet.position.x)
		if receiver == -1:
			if droplet.position.x < -droplet.radius or droplet.position.x > get_playfield_size().x + droplet.radius:
				droplets.remove_at(index)
			continue

		var merge_y := minf(_surface_y_from_volume(receiver, water_volume[receiver]), column_bottom_y[receiver])
		if droplet.position.y + droplet.radius >= merge_y:
			if droplet.volume > 0.0:
				water_volume[receiver] = minf(column_capacity[receiver], water_volume[receiver] + droplet.volume)
			droplets.remove_at(index)


func _spill_excess_as_droplets(column_x: int, excess: float) -> void:
	if excess <= 0.0:
		return

	var drop_count := int(ceil(excess * 4.0))
	if drop_count < 1:
		drop_count = 1
	var per_drop := excess / float(drop_count)
	var spawn_x := (float(column_x) + 0.5) * cell_size
	var spawn_y := maxf(0.0, _surface_y_from_volume(column_x, column_capacity[column_x]) - 6.0)

	for index in range(drop_count):
		var ratio := 0.0
		if drop_count > 1:
			ratio = float(index) / float(drop_count - 1) - 0.5
		_spawn_droplet(Vector2(spawn_x + ratio * cell_size * 0.5, spawn_y), Vector2(ratio * 40.0, -30.0), per_drop)


func _should_spill_as_droplet(source_x: int, target_x: int, transfer: float) -> bool:
	if transfer < spill_droplet_threshold:
		return false
	if not _column_is_open(source_x) or not _column_is_open(target_x):
		return false
	return _has_step_down(source_x, target_x)


func _has_step_down(source_x: int, target_x: int) -> bool:
	if not _column_is_open(source_x) or not _column_is_open(target_x):
		return false
	return column_bottom_y[source_x] < column_bottom_y[target_x] - 0.5


func _spawn_spill_droplet(source_x: int, target_x: int, volume: float) -> void:
	if volume <= 0.0:
		return

	var direction: float = 1.0
	if target_x < source_x:
		direction = -1.0
	var edge_x: float = float(source_x + 1) * cell_size if direction > 0.0 else float(source_x) * cell_size
	var spawn_x: float = edge_x + direction * 4.0
	var spawn_y: float = _effective_surface_y(source_x)
	var landing_x: float = (float(target_x) + 0.5) * cell_size
	var landing_y: float = minf(_surface_y_from_volume(target_x, water_volume[target_x]), column_bottom_y[target_x])
	_register_spill_visual(Vector2(spawn_x, spawn_y), Vector2(landing_x, landing_y), volume)

	var drop_count: int = int(ceil(volume * 8.0))
	if drop_count < 1:
		drop_count = 1
	elif drop_count > 4:
		drop_count = 4
	var per_drop: float = volume / float(drop_count)

	for index in range(drop_count):
		var ratio := 0.0
		if drop_count > 1:
			ratio = float(index) / float(drop_count - 1) - 0.5
		var start := Vector2(spawn_x + ratio * 5.0, spawn_y + absf(ratio) * 3.0)
		var initial_velocity := Vector2(direction * (45.0 + absf(ratio) * 35.0), 35.0 + float(index) * 12.0)
		_spawn_droplet(start, initial_velocity, per_drop)


func _register_spill_visual(start: Vector2, end: Vector2, volume: float) -> void:
	var width := spill_visual_width + minf(4.0, volume * 12.0)
	spill_visuals.append(SpillVisualData.new(start, end, width, spill_visual_lifetime))


func _update_spill_visuals(delta: float) -> void:
	for index in range(spill_visuals.size() - 1, -1, -1):
		var spill := spill_visuals[index]
		spill.life -= delta
		if spill.life <= 0.0:
			spill_visuals.remove_at(index)


func _clear_body_displacement() -> void:
	for x in range(body_displacement.size()):
		body_displacement[x] = 0.0
	
func _clear_edge_spills() -> void:
	for edge in range(edge_spill_flow.size()):
		edge_spill_flow[edge] = 0.0
		edge_spill_source_y[edge] = 0.0
		edge_spill_target_y[edge] = 0.0
		edge_spill_direction[edge] = 0


func _init_arrays() -> void:
	solids.clear()
	diggable.clear()
	collision_rects.clear()
	droplets.clear()

	column_floor_row.resize(columns)
	column_capacity.resize(columns)
	column_bottom_y.resize(columns)
	water_volume.resize(columns)
	body_displacement.resize(columns)
	edge_spill_flow.resize(max(columns - 1, 0))
	edge_spill_source_y.resize(max(columns - 1, 0))
	edge_spill_target_y.resize(max(columns - 1, 0))
	edge_spill_direction.resize(max(columns - 1, 0))

	for x in range(columns):
		var solid_column := PackedByteArray()
		var diggable_column := PackedByteArray()
		solid_column.resize(rows)
		diggable_column.resize(rows)
		for y in range(rows):
			solid_column[y] = 0
			diggable_column[y] = 0
		solids.append(solid_column)
		diggable.append(diggable_column)
		column_floor_row[x] = rows
		column_capacity[x] = 0.0
		column_bottom_y[x] = float(rows) * cell_size
		water_volume[x] = 0.0
		body_displacement[x] = 0.0
		if x < columns - 1:
			edge_spill_flow[x] = 0.0
			edge_spill_source_y[x] = 0.0
			edge_spill_target_y[x] = 0.0
			edge_spill_direction[x] = 0


func _build_level() -> void:
	_add_solid_rect(0, rows - 1, columns, 1, false)
	_add_solid_rect(0, 0, 1, rows, false)
	_add_solid_rect(columns - 1, 0, 1, rows, false)
	_add_solid_rect(16, 11, 1, rows - 11, false)
	_add_solid_rect(17, 20, 8, 3, true)
	_add_solid_rect(25, 21, 5, 2, true)


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


func _rebuild_column_profiles() -> void:
	for x in range(columns):
		var top_solid := rows
		for y in range(rows):
			if solids[x][y] != 0:
				top_solid = y
				break
		column_floor_row[x] = top_solid
		column_capacity[x] = float(top_solid)
		column_bottom_y[x] = float(top_solid) * cell_size
		water_volume[x] = minf(water_volume[x], column_capacity[x])
		body_displacement[x] = 0.0


func _seed_initial_water() -> void:
	for x in range(columns):
		water_volume[x] = 0.0

	for x in range(1, 16):
		if _column_is_open(x):
			water_volume[x] = minf(column_capacity[x], 11.0)


func _add_solid_rect(start_x: int, start_y: int, width: int, height: int, is_diggable: bool) -> void:
	for x in range(start_x, start_x + width):
		for y in range(start_y, start_y + height):
			if not _is_inside(x, y):
				continue
			solids[x][y] = 1
			if is_diggable:
				diggable[x][y] = 1


func _spawn_droplet(local_position: Vector2, velocity: Vector2, volume: float) -> void:
	var radius := clampf(4.0 + sqrt(maxf(volume, 0.02)) * cell_size * 0.12, 3.0, cell_size * 0.34)
	droplets.append(DropletData.new(local_position, velocity, volume, radius))


func _find_receiver_column(local_x: float) -> int:
	var base_x := clampi(floori(local_x / cell_size), 0, columns - 1)
	if _column_is_open(base_x):
		return base_x

	for distance in range(1, columns):
		var left_x := base_x - distance
		if left_x >= 0 and _column_is_open(left_x):
			return left_x
		var right_x := base_x + distance
		if right_x < columns and _column_is_open(right_x):
			return right_x

	return -1


func _top_diggable_cell(column_x: int) -> int:
	for y in range(rows):
		if diggable[column_x][y] != 0:
			return y
	return -1


func _column_is_open(column_x: int) -> bool:
	return column_x >= 0 and column_x < columns and column_capacity[column_x] > 0.0


func _should_render_column(column_x: int) -> bool:
	if not _column_is_open(column_x):
		return false
	if _effective_volume(column_x) > render_volume_threshold:
		return true
	return _column_has_adjacent_spill(column_x)


func _column_has_adjacent_spill(column_x: int) -> bool:
	if column_x > 0 and edge_spill_flow[column_x - 1] > MIN_VOLUME:
		return true
	if column_x < edge_spill_flow.size() and edge_spill_flow[column_x] > MIN_VOLUME:
		return true
	return false


func _effective_volume(column_x: int) -> float:
	return clampf(water_volume[column_x] + body_displacement[column_x] * body_displacement_factor, 0.0, column_capacity[column_x])


func _surface_y_from_volume(column_x: int, volume: float) -> float:
	if not _column_is_open(column_x):
		return float(rows) * cell_size
	return column_bottom_y[column_x] - clampf(volume, 0.0, column_capacity[column_x]) * cell_size


func _effective_surface_y(column_x: int) -> float:
	return _surface_y_from_volume(column_x, _effective_volume(column_x))


func _connection_height_y(left_x: int, right_x: int) -> float:
	return minf(column_bottom_y[left_x], column_bottom_y[right_x])


func _should_continue_surface_region(left_x: int, right_x: int) -> bool:
	if left_x < 0 or right_x < 0 or left_x >= columns or right_x >= columns:
		return false
	if not _column_is_open(left_x) or not _column_is_open(right_x):
		return false

	var bottom_gap: float = absf(column_bottom_y[left_x] - column_bottom_y[right_x])
	if bottom_gap > 0.5:
		return false

	return true


func _smooth_surface_samples(samples: PackedFloat32Array) -> PackedFloat32Array:
	var smoothed := samples.duplicate()
	for _pass in range(surface_smoothing_passes):
		var next_samples := smoothed.duplicate()
		for index in range(smoothed.size()):
			var total := smoothed[index] * 2.0
			var weight := 2.0
			if index > 0:
				total += smoothed[index - 1]
				weight += 1.0
			if index < smoothed.size() - 1:
				total += smoothed[index + 1]
				weight += 1.0
			next_samples[index] = clampf(total / weight, 0.0, float(rows) * cell_size)
		smoothed = next_samples
	return smoothed


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


func _world_to_cell(local_position: Vector2) -> Vector2i:
	return Vector2i(floori(local_position.x / cell_size), floori(local_position.y / cell_size))


func _cell_center(x: int, y: int) -> Vector2:
	return Vector2((x + 0.5) * cell_size, (y + 0.5) * cell_size)


func _cell_origin(x: int, y: int) -> Vector2:
	return Vector2(x * cell_size, y * cell_size)


func _is_inside(x: int, y: int) -> bool:
	return x >= 0 and x < columns and y >= 0 and y < rows

func _register_edge_spill(source_x: int, target_x: int, transfer: float) -> void:
	var edge: int = min(source_x, target_x)
	if edge < 0 or edge >= edge_spill_flow.size():
		return

	var direction: int = 1
	if target_x < source_x:
		direction = -1
	edge_spill_flow[edge] += transfer
	edge_spill_direction[edge] = direction
	edge_spill_source_y[edge] = _effective_surface_y(source_x)
	edge_spill_target_y[edge] = minf(_surface_y_from_volume(target_x, water_volume[target_x]), column_bottom_y[target_x])
