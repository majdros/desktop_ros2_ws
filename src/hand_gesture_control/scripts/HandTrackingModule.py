#!/usr/bin/env python3
import cv2 as cv
import mediapipe as mp
import time
from mediapipe.python.solutions import hands
from mediapipe.python.solutions import drawing_utils


class HandDetector():
    """MediaPipe-basierter Handdetektor für Echtzeit-Handerkennung und -tracking."""
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackingCon=0.5):
        self.mode = mode                    # False = Tracking zwischen Frames, True = Erkennung in jedem Frame
        self.maxHands = maxHands            # Maximale Anzahl der zu erkennenden Hände
        self.detectionCon = detectionCon    # Minimale Konfidenz für Handerkennung (0-1) 
        self.trackingCon = trackingCon      # Minimale Konfidenz für Handverfolgung (0-1)

        self.mpHands = hands
        self.hands = self.mpHands.Hands(
            static_image_mode = self.mode,
            max_num_hands = self.maxHands,
            min_detection_confidence = self.detectionCon,
            min_tracking_confidence = self.trackingCon
            )
        self.mpDraw = drawing_utils



    def findHands(self, frame, draw=False):
        """
        Erkennt Hände im Bild und zeichnet optional die Landmarken.

        Args:
            frame: Eingabebild
            draw: Wenn True, werden Handlandmarken gezeichnet
        Returns:
            Bild mit Landmarken
        """

        imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        if draw:
            if self.results.multi_hand_landmarks:
                for handLms in self.results.multi_hand_landmarks:
                    self.mpDraw.draw_landmarks(frame, handLms, self.mpHands.HAND_CONNECTIONS)
        return frame



    def findPosition(self, frame, handNum=2):
        """
        Extrahiert Pixelkoordinaten der Handlandmarken.
        Args:
            frame: Eingabebild
            handNum: Index der zu trackenden Hand
            draw: Wenn True, werden Punkte 4 und 8 markiert
        Returns:
            Liste der Landmarken-Koordinaten [[id, x, y], ...]
        """
        lmList = []
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                hand_landmarks = []

                for id, lm in enumerate(handLms.landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    hand_landmarks.append([id, cx, cy])

                lmList.append(hand_landmarks)
        return lmList



def main():
    pTime, cTime = 0, 0
    wCam, hCam = 640, 480

    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, wCam)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, hCam)

    detector = HandDetector()

    while True:
        ret, frame = cap.read()

        frame = detector.findHands(frame)
        hands_landmarks = detector.findPosition(frame)

        for hand_landmarks in hands_landmarks:
            if len(hands_landmarks) != 0:
                print (f'Hand landmarks: {hand_landmarks[4]}')

        cTime = time.time()
        fps = 1/(cTime - pTime)
        pTime = cTime

        cv.putText(frame, f'FPS: {str(int(fps))}',(10, 30), cv.FONT_HERSHEY_SCRIPT_SIMPLEX, 1, (255, 0, 255), 3)  

        cv.imshow("webcam", frame)
        if cv.waitKey(1) == ord('q'):
            break    

    cap.release()
    cv.destroyAllWindows()



if __name__=='__main__':
    main()