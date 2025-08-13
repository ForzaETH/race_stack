#!/usr/bin/env python3
def id_to_rgb_color(id: int) -> tuple:
    """
    Convert an ID to a color in RGB space.

    Args:
        id (int): The ID to convert to a color.

    Returns:
        tuple: A tuple containing the RGB values of the color.
    """
    # Use a pseudorandom function to map IDs to varied hues while keeping saturation and value constant
    # pseudo_random = lambda id: (id * 2654435761) & 0xFFFFFFFF
    def pseudo_random(id): return (id ^ (id >> 12) ^ (id << 25)) % 360
    rand = pseudo_random(id)
    hue = (rand % 360) / 360.0  # Vary hue from 0 to 1
    saturation = 1.0
    value = 1.0
    r, g, b = hsv_to_rgb(hue, saturation, value)
    return r, g, b


def hsv_to_rgb(h, s, v):
    # Convert HSV to RGB
    i = int(h * 6)
    f = (h * 6) - i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)

    if i % 6 == 0:
        return v, t, p
    elif i % 6 == 1:
        return q, v, p
    elif i % 6 == 2:
        return p, v, t
    elif i % 6 == 3:
        return p, q, v
    elif i % 6 == 4:
        return t, p, v
    else:
        return v, p, q
