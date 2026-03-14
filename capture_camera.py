import cv2

def main():
    # When routed over USB via ADB, the IP Webcam server is accessible on localhost:8080
    video_url = "http://localhost:8080/video"
    print(f"Connecting to mobile camera at {video_url}...")
    
    cap = cv2.VideoCapture(video_url)
    
    if not cap.isOpened():
        print("Error: Could not open video stream. Please check if:")
        print("1. IP Webcam app is running on your phone")
        print("2. The ADB port forwarding is active (adb forward tcp:8080 tcp:8080)")
        return

    print("Successfully connected! Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame or connection lost.")
            break
            
        cv2.imshow('USB IP-Webcam Feed', frame)
        
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
