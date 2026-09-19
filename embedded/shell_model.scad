// PGK-155 Shell Mockup for 3D Printing
// ======================================
// Scale model of 155mm artillery shell nose section for SIH booth demo.
// Designed to house STM32 Nucleo, 4x SG90 servos, IMU, GPS, and LEDs.
//
// Print Settings:
//   Material: PLA or PETG
//   Layer Height: 0.2mm
//   Infill: 20%
//   Supports: Yes (for canard slots and internal cavity)
//   Scale: 1:2 (half-size, ~77.5mm diameter)
//
// Usage: Open in OpenSCAD, render (F6), export STL, slice and print.

// --- Parameters ---
scale_factor = 0.5;  // 1:2 scale

// Real dimensions (mm)
shell_diameter = 155;
shell_length = 400;     // Nose section only (not full 800mm shell)
ogive_length = 200;     // Ogive nose cone length
body_length = 200;      // Cylindrical body section
wall_thickness = 3;     // Shell wall thickness

// Internal cavity for electronics
cavity_diameter = 48;   // 51.1mm fuze well minus clearance
cavity_depth = 180;     // Depth of internal electronics bay

// Canard slots
canard_slot_width = 2;
canard_slot_height = 25;
canard_slot_depth = 15;
canard_offset = 20;     // Distance from nose tip to canard slots

// Servo mounting holes
servo_width = 12.2;
servo_height = 22.7;
servo_depth = 23;

// LED window
led_window_width = 15;
led_window_height = 10;
led_window_offset = 120; // From nose tip

// USB cable exit hole
cable_hole_diameter = 8;

// Apply scale
sd = shell_diameter * scale_factor;
sl = shell_length * scale_factor;
ol = ogive_length * scale_factor;
bl = body_length * scale_factor;
wt = wall_thickness * scale_factor;
cd = cavity_diameter * scale_factor;
cdepth = cavity_depth * scale_factor;

module ogive_nose(diameter, length, steps=60) {
    // Tangent ogive approximation using stacked cylinders
    r = diameter / 2;
    rho = (r * r + length * length) / (2 * r);  // Ogive radius
    
    for (i = [0:steps-1]) {
        z0 = i * length / steps;
        z1 = (i + 1) * length / steps;
        
        // Ogive radius at height z
        r0 = sqrt(rho * rho - (length - z0) * (length - z0)) + r - rho;
        r1 = sqrt(rho * rho - (length - z1) * (length - z1)) + r - rho;
        
        r0_safe = max(r0, 0.5 * scale_factor);
        r1_safe = max(r1, 0.5 * scale_factor);
        
        translate([0, 0, z0])
            cylinder(h = length/steps + 0.01, r1 = r0_safe, r2 = r1_safe, $fn=64);
    }
}

module shell_body() {
    // Main outer shell
    union() {
        // Ogive nose cone
        ogive_nose(sd, ol);
        
        // Cylindrical body section
        translate([0, 0, -bl])
            cylinder(h = bl, d = sd, $fn=64);
    }
}

module internal_cavity() {
    // Hollow out the interior for electronics
    translate([0, 0, -cdepth + ol * 0.3])
        cylinder(h = cdepth, d = cd, $fn=48);
}

module canard_slots() {
    // 4 slots at 90-degree intervals for canard fins to protrude
    csw = canard_slot_width * scale_factor;
    csh = canard_slot_height * scale_factor;
    csd = canard_slot_depth * scale_factor;
    co = canard_offset * scale_factor;
    
    for (angle = [0, 90, 180, 270]) {
        rotate([0, 0, angle])
            translate([sd/2 - csd/2, -csw/2, ol - co - csh])
                cube([csd, csw, csh]);
    }
}

module servo_mount_cavities() {
    // 4 internal pockets for SG90 servos, aligned with canard slots
    sw = servo_width * scale_factor;
    sh = servo_height * scale_factor;
    sdp = servo_depth * scale_factor;
    co = canard_offset * scale_factor;
    csh = canard_slot_height * scale_factor;
    
    for (angle = [0, 90, 180, 270]) {
        rotate([0, 0, angle])
            translate([cd/2 - 2*scale_factor, -sw/2, ol - co - csh - sdp/2])
                cube([sdp/2, sw, sh]);
    }
}

module led_window() {
    // Rectangular window for RGB LED visibility
    lw = led_window_width * scale_factor;
    lh = led_window_height * scale_factor;
    lo = led_window_offset * scale_factor;
    
    translate([sd/2 - wt - 1, -lw/2, ol - lo])
        cube([wt + 2, lw, lh]);
}

module cable_exit() {
    // Hole at the bottom for USB cable
    chd = cable_hole_diameter * scale_factor;
    translate([0, 0, -bl - 1])
        cylinder(h = wt + 2, d = chd, $fn=32);
}

module mounting_screw_holes() {
    // 4 screw holes near the base for mounting to display stand
    hole_d = 3 * scale_factor;
    hole_offset = sd/2 - 5*scale_factor;
    
    for (angle = [45, 135, 225, 315]) {
        rotate([0, 0, angle])
            translate([hole_offset, 0, -bl - 1])
                cylinder(h = wt + 2, d = hole_d, $fn=16);
    }
}

// --- Assembly ---
module pgk_shell_mockup() {
    difference() {
        // Outer shell
        shell_body();
        
        // Subtract internal cavity
        internal_cavity();
        
        // Subtract canard slots
        canard_slots();
        
        // Subtract servo mount pockets
        servo_mount_cavities();
        
        // Subtract LED viewing window
        led_window();
        
        // Subtract USB cable exit
        cable_exit();
        
        // Subtract mounting holes
        mounting_screw_holes();
    }
}

// Render the shell
pgk_shell_mockup();

// --- Separate canard fin (print 4x) ---
module canard_fin() {
    // Individual canard fin with pivot hole
    fin_span = 12 * scale_factor;
    fin_chord = 18 * scale_factor;
    fin_thick = 1.5 * scale_factor;
    pivot_d = 1.5 * scale_factor;
    
    difference() {
        // Fin body (tapered)
        hull() {
            cube([fin_chord, fin_thick, 0.01]);
            translate([fin_chord * 0.3, 0, fin_span])
                cube([fin_chord * 0.5, fin_thick, 0.01]);
        }
        
        // Pivot hole at root
        translate([fin_chord * 0.3, -1, 2*scale_factor])
            rotate([-90, 0, 0])
                cylinder(h = fin_thick + 2, d = pivot_d, $fn=16);
    }
}

// Uncomment to render a single canard fin for separate printing:
// translate([sd + 20, 0, 0]) canard_fin();
