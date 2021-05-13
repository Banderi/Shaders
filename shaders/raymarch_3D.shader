shader_type spatial;

render_mode unshaded, cull_disabled, depth_test_disable;

uniform sampler2D mat_cap;

uniform sampler2D sphere_buff;

uniform int tex_res : hint_range(1, 10) = 6;

//varying vec3 sphere_arr[24];
//varying float v_index;
//
//void vertex() {
//	v_index = v_index + 1.0;
//	sphere_arr[int(v_index)] = VERTEX;
//}

// params:
// p: arbitrary point in 3D space
// c: the center of our sphere
// r: the radius of our sphere
float sdf_sphere(in vec3 p, in vec3 c, float r)
{
    return length(c - p) - r;
}

float smin(float a, float b, float k) {
	
	int type = 0;
	
	if (type == 0) {
		k = 0.45;
		float h = clamp(0.5 + 0.5*(a-b)/k, 0.0, 1.0);
		return mix(a, b, h) - k*h*(1.0-h);
	}
	else if (type == 1) {
		k = 6.925;
		float res = exp2( -k*a ) + exp2( -k*b );
    	return -log2( res )/k;
	}
	else {
		a = pow( a, k ); b = pow( b, k );
    	return pow( (a*b)/(a+b), 1.0/k );
	}
}

float sdf_capsule(vec3 p, vec3 a, vec3 b, float r)
{
	vec3 pa = p - a, ba = b - a;
	float h = clamp( dot(pa,ba)/dot(ba,ba), 0.0, 1.0 );
	float res = length(pa - ba*h) - r;
	return res;
}

float sdf_stretch(float prev_res, vec3 p, vec3 a, vec3 b, float r, float smoothness)
{
	float res = sdf_capsule(p, a, b, r * 0.5);
	float d_b = sdf_sphere(p, b, r);

	return smin(prev_res, smin(res, d_b, smoothness), smoothness);
}


float sdf_mandelbulb(in vec3 p, in float power)
{
	if (false) { // the "optimized" version - but doesn't work?
		vec3 w = p;
	    float m = dot(w,w);
	
	    vec4 trap = vec4(abs(w),m);
		float dz = 1.0;
	    
		for( int i=0; i<4; i++ )
	    {
			if (false) {
		        float m2 = m*m;
		        float m4 = m2*m2;
				dz = 8.0*sqrt(m4*m2*m)*dz + 1.0;
		
		        float x = w.x; float x2 = x*x; float x4 = x2*x2;
		        float y = w.y; float y2 = y*y; float y4 = y2*y2;
		        float z = w.z; float z2 = z*z; float z4 = z2*z2;
		
		        float k3 = x2 + z2;
		        float k2 = inversesqrt( k3*k3*k3*k3*k3*k3*k3 );
		        float k1 = x4 + y4 + z4 - 6.0*y2*z2 - 6.0*x2*y2 + 2.0*z2*x2;
		        float k4 = x2 - y2 + z2;
		
		        w.x = p.x +  64.0*x*y*z*(x2-z2)*k4*(x4-6.0*x2*z2+z4)*k1*k2;
		        w.y = p.y + -16.0*y2*k3*k4*k4 + k1*k1;
		        w.z = p.z +  -8.0*y*k4*(x4*x4 - 28.0*x4*x2*z2 + 70.0*x4*z4 - 28.0*x2*z2*z4 + z4*z4)*k1*k2;
			} else {
		        dz = 8.0*pow(sqrt(m),7.0)*dz + 1.0;
				//dz = 8.0*pow(m,3.5)*dz + 1.0;
		        
		        float r = length(w);
		        float b = 8.0*acos( w.y/r);
		        float a = 8.0*atan( w.x, w.z );
		        w = p + pow(r,8.0) * vec3( sin(b)*sin(a), cos(b), sin(b)*cos(a) );
			}
	        
	        trap = min( trap, vec4(abs(w),m) );
	
	        m = dot(w,w);
			if( m > 256.0 )
	            break;
	    }
	
	//	resColor = vec4(m,trap.yzw);
	    return 0.25*log(m)*sqrt(m)/dz;
	
	}
	
	//////
	
	p *= 0.246;
    vec3 z = p;
    float dr = 1.0;
    float r = 0.0;

    for (int i = 1; i <= 16; i++)
	{
        r = length(z);
        if (r > 2.0)
			break;
        float theta = acos(z.z / r) * power;
        float phi = atan(z.y / z.x) * power;
        float zr = pow(r, power);
        dr = pow(r, (power - 1.0)) * power * dr + 1.0;

        z = vec3(sin(theta) * cos(phi), sin(phi) * sin(theta), cos(theta)) * zr;
        z = z + p;
    }

    return 0.5 * log( r ) * r / dr;
}
float sdf_mandel_two(in vec3 p, in float power)
{
	float blocksize = 0.1;
	p = p * 0.001 + round( p / blocksize ) *  blocksize  * 0.999;
	
    vec3 z = p;
    float dr = 1.0;
    float r = 0.0;

    for (int i = 0; i < 20; i++)
    {
        r = length(z);
		if (r > 4.0)
			break;
			
        float theta = acos(z.z / r) * power;
        float phi = atan(z.y, z.x) * power;
        float zr = pow(r, power);
        dr =  pow(r, power - 1.0) * power * dr + 1.0;

		z = zr * vec3(sin(theta) * cos(phi), sin(phi) * sin(theta), cos(theta));
        z += p;
    }

    return 0.25 * log(r) * r / dr;
}
float sdf_mandel_two_orig(in vec3 p, in float power)
{	
	vec3 z = p;
    float dr = 1.0;
    float r = 0.0;
	
    for (int i = 0; i < 20 ; i++) {
        r = length(z);
        if (r > 4.0)
			break;
        
        // convert to polar coordinates
        float theta = acos(z.z / r);
        float phi = atan(z.y, z.x);
        dr =  pow(r, power - 1.0) * power * dr + 1.0;
        
        // scale and rotate the point
        float zr = pow(r, power);
        theta = theta * power;
        phi = phi * power;
        
        // convert back to cartesian coordinates
        z = zr * vec3(sin(theta) * cos(phi), sin(phi) * sin(theta), cos(theta));
        z += p;
    }

    return 0.25 * log(r) * r / dr;
}


float sdf(in vec3 p, float time)
{
	
//	return sdf_sphere(p, vec3(0,0,0), 5.0);
	
	return sdf_mandel_two(p, 2.0 * sin(time) + 4.0);
	return sdf_mandel_two_orig(p, 3.0);
	
	return sdf_mandelbulb(p, 3.0); //2.0 * sin(time) + 4.0
	
	
	
	
	
	
	float res = 10000.0;
	float sph_radius = 0.35;
	float smoothness = 1.4;
//	int tex_res = 10;
	for (int y = 0; y < tex_res; y++) {
		for (int x = 0; x < tex_res; x++) {
			vec4 pixel = texture(sphere_buff, vec2(float(x) + 0.5, float(y) + 0.5));
//			if (pixel.a != 0.0) {
				res = smin(res, sdf_sphere(p, pixel.rgb, sph_radius), smoothness);
//			}
		}
	}
	
	if (res < 1000.0)
		return res;
	
	
//	time = time + 1.7;
    float displacement = sin(5.0 * p.x) * sin(5.0 * p.y) * sin(5.0 * p.z) * 0.25;
    displacement *= cos(time);
	
//	return sdf_capsule(p, vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0), 1.0);
//	return sdf_mandelbulb(p, 2.0 * sin(time) + 4.0);
	
//	res = sdf_sphere(p, sphere_1, sph_radius);
	
	for (int n = 0; n < 24; n++) {
//		res = sdf_stretch(res, p, sphere_1, sphere_2, sph_radius, smoothness);
//		res = smin(res, sdf_sphere(p, sphere_arr[n], sph_radius), smoothness);
	}
	return res;
    return res + displacement;
}

vec3 calculate_normal(in vec3 p, float time)
{
    const vec3 small_step = vec3(0.001, 0.0, 0.0);

    float gradient_x = sdf(p + small_step.xyy, time) - sdf(p - small_step.xyy, time);
    float gradient_y = sdf(p + small_step.yxy, time) - sdf(p - small_step.yxy, time);
    float gradient_z = sdf(p + small_step.yyx, time) - sdf(p - small_step.yyx, time);

    vec3 normal = vec3(gradient_x, gradient_y, gradient_z);

    return normalize(normal);
}

vec3 ray_march(in vec3 ro, in vec3 rd, float time, out vec3 hit_point, out float total_distance_traveled, out float smallest_distance_found, out bool hit, out int steps_taken)
{
    total_distance_traveled = 0.0;
	smallest_distance_found = 1000000.0; // initial value
    const float MINIMUM_HIT_DISTANCE = 0.001;
    const float MAXIMUM_TRACE_DISTANCE = 10000.0;
	int NUMBER_OF_STEPS = 256;

    for (int i = 0; i < NUMBER_OF_STEPS; ++i)
    {
		steps_taken = i;
        vec3 current_position = ro + total_distance_traveled * rd;

        float distance_to_closest = sdf(current_position, time);
		
		if (distance_to_closest < smallest_distance_found)
			smallest_distance_found = distance_to_closest;

        if (distance_to_closest < MINIMUM_HIT_DISTANCE || i >= (NUMBER_OF_STEPS - 2)) // hit - or whatever, if out of steps return the last hit found
        {
            // Let's return a diffuse from a fake light
            vec3 normal = calculate_normal(current_position, time);

//            // For now, hard-code the light's position in our scene
//            vec3 light_position = vec3(2.0, -5.0, 3.0);
//
//            // Calculate the unit direction vector that points from the point of intersection to the light source
//            vec3 direction_to_light = normalize(current_position - light_position);
//            float diffuse_intensity = max(0.0, dot(normal, direction_to_light));
			
            hit = true;
			hit_point = current_position;
			return normal;
        }

        if (total_distance_traveled > MAXIMUM_TRACE_DISTANCE) // miss, we went too far
	        break;

        // accumulate the distance traveled thus far
        total_distance_traveled += distance_to_closest;
    }
}

void fragment() {
    // Get Cameran origin
    vec3 ro = (CAMERA_MATRIX * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
	
    // Get Ray direction (From camera to vertex)
    vec3 rd = normalize((CAMERA_MATRIX * vec4(VERTEX, 1.0)).xyz - ro);
	
	// Do the ray marching, get the info!!!
	bool hit;
	vec3 hit_point;
	float hit_distance;
	float smallest_distance_found;
	int steps_taken;
    vec3 normal = ray_march(ro, rd, TIME, hit_point, hit_distance, smallest_distance_found, hit, steps_taken);
	
	// Finally, the REAL depth converted to distance-from-camera linear value!
	float depth = textureLod(DEPTH_TEXTURE, SCREEN_UV, 0.0).r;
    vec4 upos = INV_PROJECTION_MATRIX * vec4(SCREEN_UV * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec3 pixel_position = upos.xyz / upos.w;
	float z_depth = length(pixel_position);

	// for outline debugging / sdf visualization
	float ddd = max(0.0, 1.0 - smallest_distance_found);
	ddd = pow(ddd, 25);
	float mmm = max(0.0, 1.0 - hit_distance / 100.0);
	float sss = max(0.0, float(steps_taken) / 100.0);
	sss = pow(sss, 5) + 0.5 * sss;

    if (hit && hit_distance <= z_depth) {
		
		vec3 normal_adj = (WORLD_MATRIX * (INV_CAMERA_MATRIX * vec4(normal, 0.0))).xyz;
		normal_adj = normalize(WORLD_MATRIX * (INV_CAMERA_MATRIX * (vec4((normal), 0.0)))).xyz;
		
		vec2 v_n = (normal_adj.xy / 2.0); // + vec2(-0.075, 0.45);
  		v_n.y = (v_n.y * -1.0) + 1.0;
		
//      ALBEDO = texture(mat_cap, v_n + vec2(0.5, 0.5)).rgb;
		ALBEDO = normal_adj;
//		ALBEDO = normal;
        ALPHA = 1.0;
    }
    else {
		ALBEDO = vec3(sss, mmm, ddd);
//		ALBEDO = vec3(1.0 - min(smallest_distance_found, 1.0));

//		ALBEDO = normal;
//		ALBEDO = vec3(z_depth / 500.0);
        ALPHA = 1.0;
    }
	
	
	
//	ALBEDO = vec3(sss, mmm, ddd);
	
}