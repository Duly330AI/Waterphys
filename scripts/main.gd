extends Node2D

const WaterGridType = preload("res://scripts/water_core.gd")
const BALL_SCRIPT := preload("res://scripts/ball.gd")
const INITIAL_BALL_COUNT := 20
const DIG_RADIUS := 34.0
const POUR_RADIUS := 26.0
const POUR_AMOUNT := 0.7

@onready var water: WaterGridType = $Water
@onready var terrain: StaticBody2D = $Terrain
@onready var balls: Node2D = $Balls
@onready var status_label: Label = $UI/StatusLabel

var balls_left := INITIAL_BALL_COUNT
var level_cleared := false
var is_digging := false
var is_pouring := false


func _ready() -> void:
	_build_terrain_collision()
	_update_status()


func _process(_delta: float) -> void:
	if not level_cleared and water.get_goal_fill_ratio() >= water.goal_required_ratio:
		level_cleared = true
	_update_status()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		if event.shift_pressed:
			_pour_at_cursor()
			return
		if level_cleared:
			_reset_level()
			return
		_spawn_ball()
		return

	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_RIGHT:
		is_digging = event.pressed
		if is_digging:
			_dig_at_cursor()
		return

	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_MIDDLE:
		is_pouring = event.pressed
		if is_pouring:
			_pour_at_cursor()
		return

	if event is InputEventMouseMotion:
		if is_digging:
			_dig_at_cursor()
		if is_pouring:
			_pour_at_cursor()
		return

	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_R:
			_reset_level()


func _build_terrain_collision() -> void:
	for child in terrain.get_children():
		child.queue_free()

	terrain.position = water.position
	for rect in water.get_collision_rects():
		var shape := RectangleShape2D.new()
		shape.size = rect.size

		var collision := CollisionShape2D.new()
		collision.shape = shape
		collision.position = rect.position + rect.size * 0.5
		terrain.add_child(collision)


func _spawn_ball() -> void:
	if balls_left <= 0:
		return

	var mouse_x: float = get_global_mouse_position().x
	var min_x: float = water.global_position.x + 42.0
	var max_x: float = water.global_position.x + float(water.get_playfield_size().x) - 42.0
	var spawn_x: float = clampf(mouse_x, min_x, max_x)

	var ball: Node2D = BALL_SCRIPT.new()
	ball.position = Vector2(spawn_x, water.global_position.y + 26.0)
	ball.set("water", water)
	balls.add_child(ball)

	balls_left -= 1
	_update_status()


func _reset_level() -> void:
	for child in balls.get_children():
		child.queue_free()

	balls_left = INITIAL_BALL_COUNT
	level_cleared = false
	water.reset_level()
	_update_status()


func _dig_at_cursor() -> void:
	if water.carve_circle(get_global_mouse_position(), DIG_RADIUS):
		_build_terrain_collision()


func _pour_at_cursor() -> void:
	water.add_water_blob(get_global_mouse_position(), POUR_RADIUS, POUR_AMOUNT)


func _update_status() -> void:
	var goal_percent: int = int(round(water.get_goal_fill_ratio() * 100.0))
	var lines := [
		"Linksklick: Kugel   Shift+Linksklick/Mittlere Taste: Wasser   Rechts ziehen: Graben   R: Neustart",
		"Kugeln uebrig: %d von %d   Ziel gefuellt: %d%%" % [balls_left, INITIAL_BALL_COUNT, goal_percent]
	]

	if level_cleared:
		lines.append("Ziel erreicht. Klicke oder druecke R fuer einen Neustart.")
	elif balls_left == 0:
		lines.append("Keine Kugeln mehr. Druecke R fuer einen neuen Versuch.")
	else:
		lines.append("Baue mit Rechtsklick einen Kanal durch die braune Erde. Mit Shift+Linksklick oder mittlerer Maustaste kannst du direkt Wasser testen.")

	status_label.text = "\n".join(lines)