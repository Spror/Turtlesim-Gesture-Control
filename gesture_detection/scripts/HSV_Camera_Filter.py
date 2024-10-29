import cv2 
import dearpygui.dearpygui as dpg
import numpy as np

def get_slider_values():
    h_min = dpg.get_value("Hmin")
    s_min = dpg.get_value("Smin")
    v_min = dpg.get_value("Vmin")
    h_max = dpg.get_value("Hmax")
    s_max = dpg.get_value("Smax")
    v_max = dpg.get_value("Vmax")
    return (h_min, s_min, v_min), (h_max, s_max, v_max)

if __name__ == "__main__":
   
    dpg.create_context()
    dpg.create_viewport(title='HSV Camera Filter', width=1240, height=480, resizable = False)

    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()

    data = np.flip(frame, 2)  
    data = data.ravel()  
    data = np.asfarray(data, dtype='f') 
    texture_data = np.true_divide(data, 255.0)

    if not cap.isOpened():
        print("Camera opening failure")
        exit()

    with dpg.window(label="HSV sliders" , width=600, height=480, no_move=True, pos=(0, 0), no_close=True, no_collapse= True, no_resize=True):
        dpg.add_slider_int(label="Hmin", tag="Hmin", default_value=0, max_value=179)
        dpg.add_slider_int(label="Smin", tag="Smin", default_value=0, max_value=255)
        dpg.add_slider_int(label="Vmin", tag="Vmin", default_value=0, max_value=255)
        dpg.add_slider_int(label="Hmax", tag="Hmax", default_value=179, max_value=179)
        dpg.add_slider_int(label="Smax", tag="Smax", default_value=255, max_value=255)
        dpg.add_slider_int(label="Vmax", tag="Vmax", default_value=255, max_value=255)

    with dpg.texture_registry(show=False):
        dpg.add_raw_texture(frame.shape[1], frame.shape[0], texture_data, tag="texture_tag", format=dpg.mvFormat_Float_rgb)

    with dpg.window(label="Camera View", no_move=True, pos=(600, 0), no_close=True, no_collapse= True,  no_resize=True):
        dpg.add_image("texture_tag")

    dpg.setup_dearpygui()
    dpg.show_viewport()

    while dpg.is_dearpygui_running():
        ret, frame = cap.read()
        if not ret:
            print("Frame opening failure")
            break

  
        min_hsv, max_hsv = get_slider_values()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_frame, np.array(min_hsv), np.array(max_hsv))
        filtered_frame = cv2.bitwise_and(frame, frame, mask=mask)

        data = np.flip(filtered_frame, 2)
        data = data.ravel()
        data = np.asfarray(data, dtype='f')
        texture_data = np.true_divide(data, 255.0)
        dpg.set_value("texture_tag", texture_data)
        dpg.render_dearpygui_frame()

    dpg.destroy_context()