bl_u_s = cv2.getTrackbarPos("Blue Line U_S", "Line HSV trackbars")
        bl_u_v = cv2.getTrackbarPos("Blue Line U_V", "Line HSV trackbars")
        blue_line_lower_bound = np.array([bl_l_h, bl_l_s , bl_l_v ])
        blue_line_upper_bound = np.array([bl_u_h, bl_u_s,  bl_u_v ])
