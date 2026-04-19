extends RigidBody2D
class_name BallActor

const WaterGridType = preload("res://scripts/water_core.gd")

const RADIUS := 14.0
const BUOYANCY_FORCE := 1150.0
const WATER_DRAG := 3.5
const MAX_IMPACT_DISPLACEMENT := 0.05
const IMPACT_SPEED_FOR_MAX := 950.0
const IMPACT_TRIGGER_SPEED := 120.0

var water: WaterGridType
var was_submerged := false


func _ready() -> void:
	gravity_scale = 1.0
	contact_monitor = true
	max_contacts_reported = 4
	linear_damp = 0.15
	angular_damp = 0.9
	mass = 1.35

	var shape := CircleShape2D.new()
	shape.radius = RADIUS

	var collision := CollisionShape2D.new()
	collision.shape = shape
	add_child(collision)

	var physics_material := PhysicsMaterial.new()
	physics_material.bounce = 0.05
	physics_material.friction = 0.9
	physics_material_override = physics_material

	queue_redraw()


func _physics_process(_delta: float) -> void:
	if water == null:
		return

	water.register_circle_body(global_position + Vector2(0.0, RADIUS * 0.1), RADIUS * 1.05)

	var submerged: float = water.sample_circle_fill(global_position, RADIUS)
	var is_submerged := submerged > 0.05
	if is_submerged and not was_submerged and linear_velocity.length() > IMPACT_TRIGGER_SPEED:
		var impact_ratio: float = minf(1.0, linear_velocity.length() / IMPACT_SPEED_FOR_MAX)
		var displacement_intensity: float = MAX_IMPACT_DISPLACEMENT * impact_ratio
		water.queue_impact(global_position + Vector2(0.0, RADIUS * 0.15), RADIUS * 1.1, displacement_intensity)
	was_submerged = is_submerged

	if submerged <= 0.0:
		return

	apply_central_force(Vector2.UP * BUOYANCY_FORCE * submerged)

	var drag: float = maxf(0.0, 1.0 - WATER_DRAG * submerged * get_physics_process_delta_time())
	linear_velocity *= drag
	angular_velocity *= drag


func _draw() -> void:
	draw_circle(Vector2.ZERO, RADIUS, Color(0.96, 0.77, 0.2, 1.0))
	draw_arc(Vector2.ZERO, RADIUS, -PI * 0.25, PI * 0.8, 28, Color(1, 1, 1, 0.35), 2.0)