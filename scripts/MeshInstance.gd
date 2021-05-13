#tool
extends MeshInstance

var t = 0;
onready var Mat = self.get_surface_material(0)


# Called when the node enters the scene tree for the first time.
func _ready():
	if Engine.editor_hint:
		return
	for n in range(1,25):
		var ball_node = get_node(str("../BALL", n))
		ball_node.translation = Vector3(n % 4 - 5, 10, floor(n/6))
		ball_node.get_child(1).visible = false


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	t = t + delta

	if Engine.editor_hint:
		return

	translation = $"../controller/camera".get_global_transform().origin

#	var Mat = self.get_surface_material(0)
#	Mat.set_shader_param("camera_pos", $"../CAMERA".translation)
#	$"../BALL".translation = Vector3(0, 7.5 + sin(t), 3.0)



	var n = 0
	for ball_node in get_tree().get_nodes_in_group("balls"):
		n += 1
#		var ball_node = get_tree().get_nodes_in_group("balls")[n-1]

		Mat.set_shader_param(str("sphere_", n), ball_node.translation)

		# loop through all the other balls and make them stick
		var force_overall = Vector3()
		for i in range(1,25):
			var ball_node_target = get_node(str("../BALL", i))
			if i != n: # targeting another ball other than himself

				var dist_vec = (ball_node_target.translation - ball_node.translation)
				var dist = dist_vec.length()
				var dir = dist_vec.normalized()

				var force_mult = 135.25
				var min_dist = 2.1175
				var max_dist = 3.525

				var strength = force_mult / (dist * dist + 0.001)
				if dist > max_dist:
					pass
#					strength = 0.0
				elif dist < min_dist - 0.5:
					strength = 0.0

				force_overall += dir * strength
		ball_node.add_central_force(force_overall)
