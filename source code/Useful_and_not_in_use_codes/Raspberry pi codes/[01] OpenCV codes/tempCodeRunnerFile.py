# --- Get Trackbar Positions for general HSV tuning ---
    l_h = cv2.getTrackbarPos("L - H", "HSV Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "HSV Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "HSV Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "HSV Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "HSV Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "HSV Trackbars")

    # Define bounds for the general trackbar mask
    lower_bound_blue = np.array([l_h, l_s, l_v])
    upper_bound_blue = np.array([u_h, u_s, u_v])