# Tip-Tracking
Tip Identification algortithm for a multi-acturator magnetic endoscope

This project improves the reliability of robotic arm tip tracking using two components: a Detector and a Tracker, running at different frequencies to balance accuracy and performance.

Detector Component:
Identifies the tip’s position by analyzing camera frames thorugh a U-net combined with traditional image processing. It runs less frequently due to its computational load. The detector also monitors the tracker and reinitializes it if the tip is lost, ensuring the tracker doesn't lock onto incorrect areas (e.g., edges of the image). It resets when the tip moves outside the camera’s view.

Tracker Component (CSRT Tracker):
Provides real-time frame-by-frame tracking of the tip. It’s optimized for speed but can lose track during sudden movements or visual disturbances. When the tracker fails, the detector steps in to correct the position.

