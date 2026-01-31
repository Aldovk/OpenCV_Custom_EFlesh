# Movimiento de manos y gestos
# -Aldo Guerra
import cv2
import mediapipe as mp
import serial
import time

# === BLUETOOTH ===
BT_PORT = "/dev/rfcomm0"
BAUDRATE = 57600
try:
    ser = serial.Serial(BT_PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    print(f"Conectado a {BT_PORT}")
except Exception as e:
    print("Error Bluetooth:", e)
    exit()

# === MEDIAPIPE ===
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, min_detection_confidence=0.7, min_tracking_confidence=0.7, max_num_hands=1)
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')

# === MAPEO ===
# 8=index, 12=middle, 16=ring, 20=pinky, 4=thumb
finger_tips = [8, 12, 16, 20, 4]
finger_ids = [122, 123, 124, 125, 121]
prev_state = [0] * 5
prev_gesture = None
last_gesture_time = 0
GESTURE_COOLDOWN = 1.0 # segundos de espera entre gestos

def send_command(cmd):
    ser.write((cmd + "\n").encode())
    print("Enviado:", cmd)

def is_finger_up(landmarks, tip_id):
    tip = landmarks[tip_id]
    pip = landmarks[tip_id - 2 if tip_id != 4 else 17]
    return tip.y < pip.y

def detect_gesture(state):
    thumb, index, middle, ring, pinky = state[4], state[0], state[1], state[2], state[3]
    if sum(state) == 0: return "fist"
    elif sum(state) == 5: return "open"
    elif thumb == 1 and index == 1 and middle == 0 and ring == 0 and pinky == 0: return "pinch"
    elif thumb == 1 and index == 1 and middle == 1 and ring == 0 and pinky == 0: return "ok"
    elif index == 1 and middle == 1 and thumb == 0 and ring == 0 and pinky == 0: return "peace"
    elif index == 1 and thumb == 0 and middle == 0 and ring == 0 and pinky == 0: return "point"
    elif thumb == 1 and index == 0 and middle == 0 and ring == 0 and pinky == 1: return "quit"
    else: return None

def send_gesture(gesture):
    global prev_gesture, last_gesture_time
    now = time.time()
    if gesture == prev_gesture or (now - last_gesture_time) < GESTURE_COOLDOWN:
        return
    commands = {
        "fist": "close_all", "open": "open_all", "pinch": "grip",
        "ok": "wave_hand", "peace": "close_all;open_finger2;open_finger3",
        "point": "open_finger2", "quit" : "q"
    }
    cmd = commands.get(gesture)
    if cmd:
        for c in cmd.split(";"):
            send_command(c.strip())
        print(f"Gesto: {gesture}")
        prev_gesture = gesture
        last_gesture_time = now

# === BUCLE (IGUAL QUE EL EJEMPLO) ===
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Cámara no responde")
        break

    frame1 = cv2.resize(frame, (640, 480))
    rgb = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    # CORRECTO: if results.multi_hand_landmarks:
    if results.multi_hand_landmarks:
        for hand in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame1, hand, mp_hands.HAND_CONNECTIONS)

            current_state = []
            for i, tip in enumerate(finger_tips):
                up = is_finger_up(hand.landmark, tip)
                current_state.append(1 if up else 0)

                if current_state[i] != prev_state[i]:
                    cmd = f"{'open' if up else 'close'}_finger{finger_ids[i] - 120}"
                    send_command(cmd)

            prev_state = current_state.copy()
            gesture = detect_gesture(current_state)
            if gesture:
                send_gesture(gesture)
    cv2.imshow("Hand Tracking to OpenCM", frame1)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()