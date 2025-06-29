#Distance calculation logic
---

### 🔍 **Core Formula Used**

```python
distance_cm = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / perceived_width_px
```

This is based on the **pinhole camera model** from computer vision.

---

### 🎯 What Each Term Means:

| Term                 | Explanation                                                          |
| -------------------- | -------------------------------------------------------------------- |
| `KNOWN_WIDTH_CM`     | Real-world width of the object (10 cm for WRO traffic signs)         |
| `FOCAL_LENGTH`       | Calibrated or estimated focal length of the camera in pixels         |
| `perceived_width_px` | Width of the object in the image (bounding box) — measured in pixels |

---

### 🧠 **The Logic Behind It**

From the geometry of a pinhole camera:

$$
\frac{\text{Real Width (cm)}}{\text{Image Width (px)}} = \frac{\text{Real Distance (cm)}}{\text{Focal Length (px)}}
$$

Rearranged:

$$
\text{Distance (cm)} = \frac{\text{Real Width (cm)} \times \text{Focal Length (px)}}{\text{Image Width (px)}}
$$

---

### 📏 Why This Works here

Since your traffic signs are **known and consistent in size (10 cm width)**, this formula works beautifully — assuming:

* The sign is facing the camera.
* There’s not too much tilt or perspective distortion.
* You’ve calibrated the `FOCAL_LENGTH` once using a known object at a known distance.

---

### 🎨 What About Aspect Ratio?

The aspect ratio check here:

```python
if not (1.6 <= rect_aspect <= 2.4):
    continue
```

✅ **Purpose**:
To filter out **false positives** — ensuring that the shape found in the frame **looks like the real traffic sign**, which has a **2:1 aspect ratio** (height\:width or vice versa depending on orientation).

So you're not using the aspect ratio **in the distance formula**, but rather as a **guard** before measuring distance — to avoid calculating distances to random rectangles like tiles, floor patterns, or odd shadows.

---

### 📸 About Focal Length: How Is `700` Chosen?

* The value `700.0` is not universal — it’s typically:

  * **Estimated experimentally**, by measuring an object of known width at a known distance and solving:

  $$
  \text{Focal Length (px)} = \frac{\text{Perceived Width (px)} \times \text{Distance (cm)}}{\text{Real Width (cm)}}
  $$

  * Or derived via camera calibration if you're being fancy.

📏 **Quick way to calibrate** with your 10 cm column:

* Place it exactly 50 cm from the camera.
* Measure its bounding box width on the image: e.g. 35 pixels.
* Use:

$$
\text{FOCAL_LENGTH} = \frac{35 \times 50}{10} = 175.0
$$

So your `700.0` might assume a different scale or higher-resolution camera — feel free to recalibrate!

---

### 🛣️ TL;DR Summary

* ✅ **Aspect ratio**: used as a filter to ensure you’re detecting the right-shaped object (around 2:1 like your columns).
* ✅ **Focal length**: used to convert image width in pixels into a real-world distance in centimeters.
* ✅ **Distance logic**: classic pinhole camera projection formula, ideal when you know the real object size (which WRO gives you 😎).

---

If you're planning to make it **more robust** or simulate it in Gazebo or RViz later, we can even walk through calibration and distortion handling next. Let me know!
