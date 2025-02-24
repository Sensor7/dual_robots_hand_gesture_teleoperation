




import cv2
import mediapipe as mp

import time



# Initialize MediaPipe hands module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Initialize OpenCV window
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Flip the frame for a mirror view
    frame = cv2.flip(frame, 1)
    
    # Process the frame with MediaPipe hands
    results = hands.process(frame)


    # if results.multi_hand_landmarks:
    #     for hand_landmarks in results.multi_hand_landmarks:
    #         # Extract wrist and finger positions (for gestures)
    #         wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    #         thumb = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    #         index_finger = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]

    #         # Calculate Y-coordinate of wrist to detect up/down movement
    #         wrist_y = wrist.y

    #         # Gesture detection logic:
    #         # Check if the hand is up or down based on wrist Y position
    #         if wrist_y < 0.4:  # Hand is raised
    #             print("Move shoulder up")
    #         elif wrist_y > 0.6:  # Hand is lowered
    #             print("Move shoulder down")
    #         else:
    #             # Default neutral state: shoulder stays in the middle
    #             pass

    #         # Check if the thumb is moving right or left for base control
    #         if thumb.x > index_finger.x:  # Thumb is to the right
    #             print("Move base left")
    #         elif thumb.x < index_finger.x:  # Thumb is to the left
    #             print("Move base right")

    #         # Check if thumb and index finger are spread for gripper open
    #         if abs(thumb.x - index_finger.x) > 0.1:
    #             print("Open gripper")
    #         else:
    #             print("Close gripper")

    #         # Draw landmarks on the frame (optional for debugging)
    #         mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Display the frame
    cv2.imshow("Gesture Controlled Arm", frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()


