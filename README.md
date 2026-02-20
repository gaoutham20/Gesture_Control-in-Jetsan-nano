# Gesture_Control-in-Jetsan-nano
import cv2
import mediapipe as mp
import subprocess
import time
import math

# ==============================
# CONFIGURATION
# ==============================
FRAME_CONFIRMATION = 4
COOLDOWN_TIME = 0.6
SWIPE_THRESHOLD = 70
PINCH_THRESHOLD = 0.05
ANALOG_SENSITIVITY = 300

# ==============================
# SYSTEM COMMANDS
# ==============================
def run_cmd(cmd):
    subprocess.Popen(cmd, shell=True)

def get_volume():
    try:
        out = subprocess.check_output(
            "pactl get-sink-volume @DEFAULT_SINK@ | grep -o '[0-9]*%' | head -1",
            shell=True)
        return int(out.decode().strip().replace('%',''))
    except:
        return 50

def set_volume(percent):
    percent = max(0, min(150, percent))
    run_cmd(f"pactl set-sink-volume @DEFAULT_SINK@ {percent}%")

def toggle_mute():
    run_cmd("pactl set-sink-mute @DEFAULT_SINK@ toggle")

# ==============================
# MEDIAPIPE SETUP
# ==============================
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.8,
    min_tracking_confidence=0.8
)

def fingers_up(hand_landmarks):
    lm = hand_landmarks.landmark
    fingers = []

    fingers.append(1 if lm[4].x < lm[3].x else 0)

    tips = [8, 12, 16, 20]
    pips = [6, 10, 14, 18]
    for tip, pip in zip(tips, pips):
        fingers.append(1 if lm[tip].y < lm[pip].y else 0)

    return fingers

def distance(lm1, lm2):
    return math.sqrt((lm1.x - lm2.x)**2 + (lm1.y - lm2.y)**2)

# ==============================
# MAIN
# ==============================
cap = cv2.VideoCapture(0)

last_trigger_time = 0
gesture_counter = 0
current_gesture = None
start_pos = None
action_text = "Waiting..."

pinch_active = False
pinch_start_x = 0
pinch_start_y = 0
start_volume = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        fingers = fingers_up(hand_landmarks)
        lm = hand_landmarks.landmark
        cx, cy = int(lm[8].x * w), int(lm[8].y * h)

        pinch_dist = distance(lm[4], lm[8])

        # ================= PINCH CONTROL =================
        if pinch_dist < PINCH_THRESHOLD:

            if not pinch_active:
                pinch_active = True
                pinch_start_x = cx
                pinch_start_y = cy
                start_volume = get_volume()

            dx = cx - pinch_start_x
            dy = pinch_start_y - cy

            # Horizontal → Next / Previous
            if abs(dx) > SWIPE_THRESHOLD and time.time()-last_trigger_time > COOLDOWN_TIME:
                if dx > 0:
                    run_cmd("playerctl next")
                    action_text = "NEXT TRACK"
                else:
                    run_cmd("playerctl previous")
                    action_text = "PREVIOUS TRACK"

                pinch_start_x = cx
                last_trigger_time = time.time()

            # Vertical → Smooth Analog Volume
            else:
                delta = dy / ANALOG_SENSITIVITY
                new_vol = start_volume + (delta * 100)
                set_volume(new_vol)
                action_text = "VOLUME (Analog)"

        else:
            pinch_active = False

            count = fingers.count(1)

            # ============ STATIC GESTURES ============
            if fingers == [1,1,1,1,1] and time.time()-last_trigger_time>COOLDOWN_TIME:
                run_cmd("playerctl play")
                action_text="PLAY"
                last_trigger_time=time.time()

            elif fingers == [0,0,0,0,0] and time.time()-last_trigger_time>COOLDOWN_TIME:
                run_cmd("playerctl pause")
                action_text="PAUSE"
                last_trigger_time=time.time()

            elif fingers == [0,1,1,0,0] and time.time()-last_trigger_time>COOLDOWN_TIME:
                toggle_mute()
                action_text="MUTE"
                last_trigger_time=time.time()

            elif fingers == [1,0,0,0,1] and time.time()-last_trigger_time>COOLDOWN_TIME:
                run_cmd("xdotool key f")
                action_text="FULLSCREEN"
                last_trigger_time=time.time()

            # ============ SWIPE GESTURES ============
            if count in [1,3,4]:

                if start_pos is None:
                    start_pos = (cx,cy)

                dx_swipe = cx - start_pos[0]

                if abs(dx_swipe) > SWIPE_THRESHOLD and time.time()-last_trigger_time>COOLDOWN_TIME:

                    # 1 Finger → Seek
                    if count == 1:
                        if dx_swipe > 0:
                            run_cmd("playerctl position 10+")
                            action_text="SEEK +10s"
                        else:
                            run_cmd("playerctl position 10-")
                            action_text="SEEK -10s"

                    # 3 Fingers → Subtitle
                    elif count == 3:
                        if dx_swipe > 0:
                            run_cmd("xdotool key v")
                            action_text="SUBTITLE NEXT"
                        else:
                            run_cmd("xdotool key Shift+v")
                            action_text="SUBTITLE PREV"

                    # 4 Fingers → Audio
                    elif count == 4:
                        if dx_swipe > 0:
                            run_cmd("xdotool key b")
                            action_text="AUDIO NEXT"
                        else:
                            run_cmd("xdotool key Shift+b")
                            action_text="AUDIO PREV"

                    start_pos=(cx,cy)
                    last_trigger_time=time.time()

            else:
                start_pos=None

    else:
        pinch_active=False
        action_text="Waiting..."

    cv2.putText(frame,f"Action: {action_text}",(10,60),
                cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)

    cv2.imshow("Gesture Media Control",frame)

    if cv2.waitKey(1)&0xFF==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
