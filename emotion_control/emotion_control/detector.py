"""DeepFace adapter. Import heavy dependencies only in the inference worker."""

import math


def select_face(results):
    """Use the largest face; never promote an allowed runner-up emotion."""
    if isinstance(results, dict):
        results = [results]
    if not results:
        return None, 0.0, None
    face = max(results, key=lambda f: f.get('region', {}).get('w', 0)
               * f.get('region', {}).get('h', 0))
    scores = face.get('emotion', {})
    if not scores:
        return None, 0.0, face.get('region')
    label = max(scores, key=scores.get)
    confidence = float(scores[label]) / 100.0
    if not math.isfinite(confidence) or not 0 <= confidence <= 1:
        return None, 0.0, None
    return label, confidence, face.get('region')


class Detector:
    def __init__(self):
        from deepface import DeepFace
        self.backend = DeepFace
        self.backend.build_model(model_name='Emotion', task='facial_attribute')

    def analyze(self, frame):
        try:
            results = self.backend.analyze(
                img_path=frame, actions=['emotion'], detector_backend='opencv',
                enforce_detection=True, silent=True,
            )
        except ValueError as exc:
            # DeepFace raises ValueError when no face is detected. Other model
            # or dependency errors must remain visible in the worker status.
            if 'face could not be detected' in str(exc).lower():
                return None, 0.0, None
            raise
        return select_face(results)
