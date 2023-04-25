from cv2 import VideoCapture
from queue import Queue
from threading import Thread

class BufferlessVideoCapture(object):
    def __init__(self, video_source):
        self.cap = VideoCapture(video_source)
        self.frame_queue = Queue()
        t = Thread(target=self.run)
        t.daemon = True
        t.start()

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except Queue.Empty:
                    pass
            self.frame_queue.put(frame)

    def read(self):
        return self.frame_queue.get()

    def release(self):
        self.cap.release()
