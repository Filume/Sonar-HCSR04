import RPi.GPIO as GPIO
import time
import matplotlib
matplotlib.use("TkAgg") #Using TkAgg backend for matplotlib because RPi does not support Cairo
import numpy as np
import matplotlib.pyplot as plt

# GPIO setup
SERVO_PIN = 13
TRIG_PIN = 18
ECHO_PIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

def servo_angle(angle):
    # SG90: 0 degrees ≈ 2.5% duty, 180 degrees ≈ 12.5% duty
    duty = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.1) # Allow servo to reach position
    servo.ChangeDutyCycle(0)

def measure_distance():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05) # Pause to ensure the echo dissipates
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    
    pulse_start, pulse_end = time.time(), time.time()
    
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    
    #HCSRO4 typically measures from 2 cm to 200 cm, so we constrain the distance to that range
    if 2 < distance < 200:
        distance = round(distance, 1)
    elif distance <= 2:
        distance = 2
    elif distance >= 200:
        distance = 200
    
    return distance

# Blitting Manager Class
class BlitManager:
    def __init__(self, canvas, animated_artists=()):
        self.canvas = canvas
        self._bg = None  # Stores the background (static part of the plot)
        self._artists = []  # List of animated artists to update
        
        # Add all animated artists (e.g., scatter, text)
        for a in animated_artists:
            self.add_artist(a)
        
        # Connect to the draw event to capture the background when the figure is drawn
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)
    
    def on_draw(self, event):
        # Called when the figure is drawn (or manually)
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        # Save a copy of the background (everything except animated artists)
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()  # Draw the animated artists on top
    
    def add_artist(self, art):
        # Add an artist (e.g., scatter, text) to be animated
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)  # Mark as animated so it's not drawn in the static background
        self._artists.append(art)
    
    def _draw_animated(self):
        # Draw all animated artists on the figure
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)
    
    def update(self):
        # Efficiently update the animated artists:
        cv = self.canvas
        fig = cv.figure
        
        if self._bg is None:
            # If background not captured yet, do a full draw
            self.on_draw(None)
        else:
            # Restore the background (static plot)
            cv.restore_region(self._bg)
            # Draw only the animated artists (fast!)
            self._draw_animated()
            # Blit (copy) the updated region to the screen
            cv.blit(fig.bbox)
            # Flush events to update the GUI
            cv.flush_events()

# Prepare plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)
#Limit the polar plot to a semi-circle facing downwards and update it from right to left
ax.set_thetamin(0)
ax.set_thetamax(180)
ax.set_theta_zero_location('W')
ax.set_theta_direction(1)
ax.set_ylim(0, 200)
ax.set_title("Radar (HC-SR04 + SG90)")

# Create animated artists
sc = ax.scatter([], [], c=[], cmap='hsv', s=30, vmin=0, vmax=200)
text_info = plt.gcf().text(0.5, 0.2, "", ha='center', fontsize=12)

# Initialize blitting manager
bm = BlitManager(fig.canvas, [sc, text_info])

plt.show(block=False)
plt.pause(0.1)

servo_angle(0)
angle_degrees = np.arange(0, 181, 4)
num_points = len(angle_degrees)

try:
    while True:
        angles = np.zeros(num_points)
        distances = np.zeros(num_points)
        
        for i, angle in enumerate(angle_degrees):
            servo_angle(angle)
            dist = measure_distance()
            
            angles[i] = np.deg2rad(angle)
            distances[i] = dist
            
            # **KEY FIX**: Ensure proper NumPy array format for set_offsets
            if i == 0:
                # For single point, create proper 2D array
                offsets = np.array([[angles[0], distances[0]]])
            else:
                # For multiple points, stack properly
                offsets = np.column_stack((angles[:i+1], distances[:i+1]))
            
            sc.set_offsets(offsets)
            sc.set_array(distances[:i+1])
            
            text_info.set_text(f"Kąt: {angle} Dystans: {dist:.1f} cm")
            bm.update()
        
        # Clear for next sweep - use empty NumPy array
        sc.set_offsets(np.empty((0, 2)))  # Empty N×2 array
        sc.set_array(np.array([]))        # Empty 1D array
        text_info.set_text("")
        bm.update()

except KeyboardInterrupt:
    pass
finally:
    servo.stop()
    GPIO.cleanup()
    plt.ioff()
    plt.show()
