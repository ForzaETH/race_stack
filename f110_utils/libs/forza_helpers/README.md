# `color_utils.py`

Utility functions to convert object IDs to distinct colors in RGB format based on HSV values.

## Functions
  - `id_to_rgb_color(id)` – Converts an ID to an RGB color.
  - `hsv_to_rgb(h, s, v)` – Converts HSV values to RGB values.

## Usage
Import the package:

```python
from id_to_color.id_to_color import id_to_rgb_color
```

Convert id to a color:
```python
r, g, b = id_to_rgb_color(id)
```