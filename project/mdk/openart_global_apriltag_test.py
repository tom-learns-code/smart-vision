# OpenART Plus / OpenMV IDE live test.
# Goal: detect AprilTag landmarks and estimate a rough global pose.
#
# Run this script with the green Run button first. Do not burn it to flash yet.
# Put TAG36H11 AprilTags at known positions, then edit TAG_MAP below.

import sensor
import image
import time
import math


# ---------------- User settings ----------------

# Printed AprilTag black square side length, in millimeters.
TAG_SIZE_MM = 80.0

# Camera/lens values from OpenMV AprilTag examples. Good enough for first test.
LENS_MM = 2.8
SENSOR_W_MM = 3.984
SENSOR_H_MM = 2.952
IMG_W = 160
IMG_H = 120

# Tag map: id -> (world_x_mm, world_y_mm, world_yaw_deg)
# world_yaw_deg means the tag's "up" direction in your map.
# Start with one tag at map origin. Add more after tag 0 works.
TAG_MAP = {
    0: (0.0, 0.0, 0.0),
    1: (500.0, 0.0, 0.0),
    2: (0.0, 500.0, 90.0),
}

# Mount/calibration knobs. Change these after you compare with real movement.
CAMERA_YAW_OFFSET_DEG = 0.0
CAMERA_X_OFFSET_MM = 0.0     # camera position relative to robot center, +right
CAMERA_Y_OFFSET_MM = 0.0     # camera position relative to robot center, +forward

# For a downward camera looking at floor tags, keep this False.
# For a forward camera looking at vertical wall tags, try True.
FRONT_CAMERA_WALL_TAG_MODE = False

# Sign knobs. If x/y changes in the wrong direction, flip these to -1.0.
REL_X_SIGN = 1.0
REL_Y_SIGN = 1.0
YAW_SIGN = -1.0


# ---------------- Camera setup ----------------

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

F_X = (LENS_MM / SENSOR_W_MM) * IMG_W
F_Y = (LENS_MM / SENSOR_H_MM) * IMG_H
C_X = IMG_W * 0.5
C_Y = IMG_H * 0.5

clock = time.clock()


# ---------------- Helpers ----------------

def rad_to_deg(rad):
    return (180.0 * rad) / math.pi


def wrap_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def translation_to_mm(translation, tag_size_mm):
    # OpenMV MAVLink example uses this conversion for AprilTag translation.
    return ((translation * 100.0) * tag_size_mm) / 210.0


def rotate_2d(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def estimate_pose(tag):
    tag_id = tag.id()
    if tag_id not in TAG_MAP:
        return None

    tag_world_x, tag_world_y, tag_world_yaw = TAG_MAP[tag_id]

    tx_mm = translation_to_mm(tag.x_translation(), TAG_SIZE_MM)
    ty_mm = translation_to_mm(tag.y_translation(), TAG_SIZE_MM)
    tz_mm = translation_to_mm(tag.z_translation(), TAG_SIZE_MM)
    rz_deg = rad_to_deg(tag.z_rotation())

    # Relative tag position in robot/camera 2D frame.
    # Downward camera: use tag x/y in image plane.
    # Forward camera: use horizontal x and distance z.
    rel_x = REL_X_SIGN * tx_mm
    if FRONT_CAMERA_WALL_TAG_MODE:
        rel_y = REL_Y_SIGN * tz_mm
    else:
        rel_y = REL_Y_SIGN * ty_mm

    # Initial yaw estimate. This sign may need calibration on your car.
    robot_yaw = wrap_deg(tag_world_yaw + (YAW_SIGN * rz_deg) + CAMERA_YAW_OFFSET_DEG)

    # Tag position relative to robot center.
    rel_x += CAMERA_X_OFFSET_MM
    rel_y += CAMERA_Y_OFFSET_MM

    # Robot world position = tag world position - relative vector in world frame.
    dx_world, dy_world = rotate_2d(rel_x, rel_y, robot_yaw)
    robot_x = tag_world_x - dx_world
    robot_y = tag_world_y - dy_world

    dist_mm = math.sqrt((tx_mm * tx_mm) + (ty_mm * ty_mm) + (tz_mm * tz_mm))

    return (tag_id, robot_x, robot_y, robot_yaw, dist_mm, tx_mm, ty_mm, tz_mm, rz_deg)


# ---------------- Main loop ----------------

while True:
    clock.tick()
    img = sensor.snapshot()

    tags = img.find_apriltags(
        families=image.TAG36H11,
        fx=F_X,
        fy=F_Y,
        cx=C_X,
        cy=C_Y,
    )

    known_pose = None
    best_tag = None

    for tag in tags:
        img.draw_rectangle(tag.rect(), color=(255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))
        img.draw_string(tag.cx(), tag.cy(), str(tag.id()), color=(255, 255, 0))

        pose = estimate_pose(tag)
        if pose is not None:
            if best_tag is None or (tag.w() * tag.h()) > (best_tag.w() * best_tag.h()):
                best_tag = tag
                known_pose = pose

    if known_pose:
        tag_id, x_mm, y_mm, yaw_deg, dist_mm, tx, ty, tz, rz = known_pose
        print("#VIS,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f" %
              (tag_id, x_mm, y_mm, yaw_deg, dist_mm, tx, ty, tz, rz, clock.fps()))
    else:
        print("#VIS,NONE,%.1f" % clock.fps())
