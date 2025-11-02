def response_planner(distance, threshold=10.0):
    """
    Simple response policy:
    - If too close, slow down.
    - Otherwise, maintain speed.
    """
    if distance < threshold:
        return [0.0, 0.2]  # brake
    return [0.0, 1.0]  # accelerate
