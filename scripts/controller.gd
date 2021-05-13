extends Spatial

onready var cam = get_node("camera")

var up = Vector3(0, 1, 0)

var offset = Vector3(0, 1, 0) # height off of the ground
var target = Vector3(-4, 0, 0) # interpolation target
var lookat = target + offset

var locked = false
var closeup = false

var max_height = 0.1 * PI
var min_height = - 0.5 * PI
var phi = 1.0
var theta = -0.5

var zoom = 17 # note: zoom works in reverse - lower value means closer to target! yes I know that's not how zoom works
var zoom_curve = 1
var zoom_target = zoom # zoom's interpolation target
var max_zoom = 50

#var mouse_on_ui = false

var pick = {}
var hl_prop = null
var command_point = Vector3()
var max_height_diff = 0.5

func zoom(z):
	zoom_target += z
	zoom_target = max(zoom_target, 0.5)
	zoom_target = min(zoom_target, max_zoom)
func move_naive(x, y, s = 1.0):
	var r = Vector3(x, 0, y).rotated(up, phi)
	target += r * s * (0.75 + zoom_curve * 0.95) # compensate for zoom levels
func move_pan(x, y, s = 1.0):
	var r = Vector3(x, 0, y).rotated(Vector3(1, 0, 0), theta+0.5*PI).rotated(up, phi)
	target += r * s * (0.75 + zoom_curve * 0.95) # compensate for zoom levels

#func center():
#	target = game.player.pos

func update_raycast():
	var masks = 1 + 4 + 8
	var proj_origin = cam.project_ray_origin(get_viewport().get_mouse_position())
	var proj_normal = cam.project_ray_normal(get_viewport().get_mouse_position())

	# first raycast - from 1000 units behind camera to 1000 in front
	var from = proj_origin - proj_normal * 1000
	var to = from + proj_normal * 2000
#	var result = game.space_state.intersect_ray(from, to, [], masks, true, true)

	# raycast twice because Godot is too cool to recognize collision normals, even for concave shapes >:(
#	if result:
#		if ((proj_origin - result.position).normalized() - proj_normal).length() < 1:
#			from = proj_origin # if collision was behind camera, do again from the camera
#		else:
#			from = result.position + proj_normal * 0.1 # if not, do from the first collision point onwards
#		to = from + proj_normal * 1000
#		var result2 = game.space_state.intersect_ray(from, to, [], masks, true, true)
#
#		# final, correct collision point!
#		if result2:
#			pick = result2.duplicate()
#		else:
#			pick = result.duplicate()
#
#		# mouse hover over props
#		get_tree().call_group_flags(2, "props", "highlight", false)
#		hl_prop = null
#		var m = pick.collider.collision_layer
#		debug.t1.text = str(m)
#		match m:
#			4, 8:
#				hl_prop = pick.collider.get_parent()
#				hl_prop.highlight(true)
##			8:
##				hl_prop = pick.collider
##				pick.collider.highlight(true)
#		update_cursor()
#	else:
#		pick = {}
func update_cursor():
	pass

func _input(event):
#	if UI.paused:
#		return

	# mouse movement!!
	if event is InputEventMouseMotion:
		if Input.is_action_pressed("camera_orbit"): # orbit camera
			if Input.is_action_pressed("camera_zoomdrag"): # drag zoom (ctrl + orbit)
				zoom(1.5 * event.relative.y * 0.05)
			elif Input.is_action_pressed("camera_drag") && !locked: # pan camera (shift + orbit)
				move_pan(-event.relative.x * 0.01, -event.relative.y * 0.01)
			else:
				phi -= event.relative.x * 0.5 * 0.0075
				theta -= event.relative.y * 0.5 * 0.0075
				while phi < 0:
					phi += 2 * PI
				while phi > 2 * PI:
					phi -= 2 * PI
				theta = max(min_height, theta)
				theta = min(max_height, theta)

	# zooming only if CTRL is pressed - otherwise use the keybind to scroll items
	if !closeup:
		if Input.is_action_pressed("camera_zoomin"):
			zoom(-1.5)
		if Input.is_action_pressed("camera_zoomout"):
			zoom(1.5)

#		if Input.is_action_pressed("camera_center"):
#			center()
		if Input.is_action_just_pressed("camera_follow"):
			locked = !locked

	# if mouse is on inv. panel, don't move player
#	if UI.handle_input > 0:
#		return

#	if Input.is_action_just_released("player_command") && !pick.empty(): # send actor on an adventure!
#		if hl_prop != null:
#			game.player.reach_prop(hl_prop)
#		else:
#			command_point = pick.position
#			command_point += pick.normal * 0.2 # offset by normal
#			game.player.travel(command_point)
#	if Input.is_action_just_released("player_cancel"): # cancel adventure....
#		game.player.cancel()
#	if Input.is_action_just_pressed("character_switch"): # switch available character
#		game.switch_character()
#		center()
	if Input.is_action_just_released("debug_quit"): # cancel adventure....
		get_tree().quit()
	if Input.is_action_just_released("debug_reload"): # cancel adventure....
#		get_tree().reload_current_scene()
		for n in range(1,25):
			var ball_node = get_node(str("../BALL", n))
			ball_node.translation = Vector3(n % 4 - 5, 10, floor(n/6))
			ball_node.get_child(1).visible = false
			ball_node.set_linear_velocity(Vector3())

#func _physics_process(delta):
#	if !UI.paused && UI.handle_input == 0:
#		update_raycast()

func _process(delta):
#	if !UI.paused:
#		if Input.is_action_pressed("shoot"):
#			match game.player.state:
#				Actor.states.turret:
#					game.player.prop_inuse.fire()

	var s = 0.25
	if !locked:
		if Input.is_action_pressed("move_up"):
			move_naive(0, -s)
		if Input.is_action_pressed("move_down"):
			move_naive(0, s)
		if Input.is_action_pressed("move_left"):
			move_naive(s, 0)
		if Input.is_action_pressed("move_right"):
			move_naive(-s, 0)
#	elif locked:
#		center()

#	if closeup:
#		lookat += delta * (game.player.pos + offset - lookat) * 20
#		zoom += delta * (6 - zoom) * 20
#	else:
	lookat += delta * (target + offset - lookat) * 20
	zoom += delta * (zoom_target - zoom) * 20
	zoom_curve = 0.2 + 0.0075 * zoom * zoom

	# update camera transform
	set_transform(Transform(
		Transform(Basis()).rotated(Vector3(1, 0, 0), theta).rotated(up, phi).scaled(Vector3(zoom_curve, zoom_curve, zoom_curve)).basis,
		Vector3(0,0,0)))
	global_translate(lookat)

	###

#	if !pick.empty():
#		debug.point(pick.position, Color(1, 0, 0))
#		debug.line(pick.position, pick.position + pick.normal, Color(1, 0, 0))
#
#	debug.point(command_point, Color(0,1,0))

#func _ready():
#	game.controller = self
