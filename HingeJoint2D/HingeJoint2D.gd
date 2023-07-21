extends PinJoint2D
class_name HingeJoint2D

@export var use_limit:bool = true
@export var lower_limit:float = 0.0
@export var uppper_limit:float = 0.0
@export var relaxation_factor:float = 1.0
@export var bias_factor:float =  0.3

func _physics_process(delta):
	setup(delta)
	solve(delta)
	pass

var body_a:PhysicsBody2D
var body_b:PhysicsBody2D

var m_correction:float = 0.0
var m_limitSign:float = 0.0
var m_solveLimit:bool = false
var m_accLimitImpulse:float = 0.0

var m_kHinge:float = 0.0

func setup(_delta):
	body_a = get_node(node_a)
	body_b = get_node(node_b)
	
	var hinge_angle:float = get_hinge_angle();
	
	m_correction = 0.0
	m_limitSign = 0.0
	m_solveLimit = false
	m_accLimitImpulse = 0.0

	if use_limit and lower_limit <= uppper_limit:
		if hinge_angle <= lower_limit:
			m_correction = lower_limit - hinge_angle
			m_limitSign = 1.0
			m_solveLimit = true
		elif hinge_angle >= uppper_limit:
			m_correction = uppper_limit - hinge_angle
			m_limitSign = -1.0
			m_solveLimit = true

	m_kHinge = 1.0 / (compute_angular_impulse_denominator(body_a) + compute_angular_impulse_denominator(body_b));
func solve(delta):
	var angVelA:float = 0.0
	var angVelB:float = 0.0
	if body_a is RigidBody2D:
		angVelA = body_a.angular_velocity
	elif body_a is StaticBody2D:
		angVelA = body_a.constant_angular_velocity
	if body_b is RigidBody2D:
		angVelB = body_b.angular_velocity
	elif body_b is StaticBody2D:
		angVelB = body_b.constant_angular_velocity

	# solve limit
	if m_solveLimit:
		# (angVelB - angVelA)에 -가 붙은 이유는 축을 Vector3(0, 0, -1)로 뒀을 때를 가정한 코드라서
		var amplitude:float = (-(angVelB - angVelA) * relaxation_factor + m_correction * (1.0 / delta) * bias_factor) * m_limitSign

		var impulseMag:float = amplitude * m_kHinge

		# Clamp the accumulated impulse
		var temp:float = m_accLimitImpulse
		m_accLimitImpulse = maxf(m_accLimitImpulse + impulseMag, 0.0)
		impulseMag = m_accLimitImpulse - temp

		var impulse:float = - impulseMag * m_limitSign  # -가 붙은 이유는 축을 Vector3(0, 0, -1)로 뒀을 때를 가정한 코드라서
		if body_a is RigidBody2D:
			body_a.apply_torque_impulse(impulse)
		if body_b is RigidBody2D:
			body_b.apply_torque_impulse(-impulse)

func get_hinge_angle() -> float:
	var center:float = (lower_limit + uppper_limit) * 0.5
	return wrapf(body_b.rotation - body_a.rotation, -PI + center, PI + center)
func compute_angular_impulse_denominator(body) -> float:
	if body is RigidBody2D:
		# 결국 inv_inertia 반환하면 됨
		return PhysicsServer2D.body_get_direct_state(body.get_rid()).inverse_inertia
	
	printerr("RigidBody2D 이외도 받아야하나...음...")
	return 1.0
