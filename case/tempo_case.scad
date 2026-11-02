// ============================================================
//  Tempo Sensor — корпус для XIAO nRF52840 Sense + LiPo 402030
//  OpenSCAD. Выберите mount и part, F6 → Export STL.
// ============================================================

mount = "glove";   // "glove" — две петли под ремешок-липучку 20 мм (на перчатку)
                   // "shaft" — ложе под шафт + 2 стяжки (под грипом)
part  = "both";    // "both" | "base" | "lid"

// ---------- Внутренний объём (плата 21×18×~4 поверх батареи 4×20×30) ----------
in_x = 22;     // ширина
in_y = 32;     // длина
in_z = 10;     // высота стопки: батарея 4.5 + плата с деталями 4 + запас
wall = 1.6;
floor_t = 1.6;
r = 2.5;       // скругление углов

// ---------- Окно USB-C (плата лежит сверху, разъём на коротком торце +Y) ----------
usb_w = 10;
usb_z0 = 5.5;  // низ окна над полом (батарея под платой)

// ---------- Крышка ----------
lid_t = 1.6;
lip_h = 2.2;
tol = 0.35;    // зазор посадки бортика

// ---------- Крепление "glove" ----------
strap_w = 21;  // ширина ремешка (20 мм + запас)
strap_t = 2.6; // толщина ремешка
ear_w = 6;     // вылет петли от стенки

// ---------- Крепление "shaft" ----------
shaft_d = 15;  // диаметр шафта под грипом (обычно 14–16 мм)
tie_w = 4.5;   // ширина стяжки
tie_t = 2;     // толщина стяжки

// ============================================================
$fn = 48;
ox = in_x + 2*wall;
oy = in_y + 2*wall;
oz = floor_t + in_z;

module rbox(x, y, z, rr) {
    linear_extrude(z) offset(rr) offset(-rr) square([x, y], center = true);
}

module cavity() {
    translate([0, 0, floor_t]) rbox(in_x, in_y, in_z + 1, 1.5);
}

module usb_window() {
    // сквозь стенку +Y, от usb_z0 до верха (крышка закрывает остаток)
    translate([-usb_w/2, in_y/2 - 1, floor_t + usb_z0])
        cube([usb_w, wall + 2, in_z]);
}

module led_hole() {
    // отверстие 2 мм в крышке над индикатором (угол платы у USB)
    translate([6, in_y/2 - 6, -1]) cylinder(d = 2, h = lid_t + 2);
}

module glove_ears() {
    for (s = [-1, 1])
        translate([s * (ox/2 + ear_w/2 - 0.01), 0, 0])
        difference() {
            rbox(ear_w + 0.02, strap_w + 5, strap_t + 2*1.6, 1);
            translate([-ear_w, -strap_w/2, 1.6]) cube([2*ear_w, strap_w, strap_t]);
        }
}

module shaft_cradle() {
    cradle_h = shaft_d/2 + 1 + tie_t + 2;
    translate([0, 0, -cradle_h])
    difference() {
        rbox(ox, oy, cradle_h + 0.01, r);
        // ложе под шафт вдоль Y
        translate([0, 0, 0]) rotate([90, 0, 0]) cylinder(d = shaft_d, h = oy + 2, center = true);
        // два паза под стяжки
        for (y = [-10, 10])
            translate([-ox/2 - 1, y - tie_w/2, shaft_d/2 + 1]) cube([ox + 2, tie_w, tie_t]);
    }
}

module base() {
    difference() {
        rbox(ox, oy, oz, r);
        cavity();
        usb_window();
    }
    if (mount == "glove") glove_ears();
    if (mount == "shaft") shaft_cradle();
}

module lid() {
    difference() {
        union() {
            rbox(ox, oy, lid_t, r);
            translate([0, 0, lid_t - 0.01])
                rbox(in_x - 2*tol, in_y - 2*tol, lip_h, 1.2);
        }
        // вырез в бортике над USB-окном
        translate([-usb_w/2 - 0.5, in_y/2 - 4, lid_t - 0.02]) cube([usb_w + 1, 5, lip_h + 1]);
        led_hole();
        // внутренняя выборка бортика, чтобы не давить на детали платы
        translate([0, 0, lid_t + 0.6]) rbox(in_x - 2*tol - 2.4, in_y - 2*tol - 2.4, lip_h, 1);
    }
}

if (part == "base" || part == "both") base();
if (part == "lid"  || part == "both") translate([ox + 12, 0, 0]) lid();
