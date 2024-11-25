import cv2 
import dearpygui.dearpygui as dpg
import numpy as np

def get_slider_values():
    Y_min = dpg.get_value("Ymin")
    Cb_min = dpg.get_value("Cbmin")
    Cr_min = dpg.get_value("Crmin")
    Y_max = dpg.get_value("Ymax")
    Cb_max = dpg.get_value("Cbmax")
    Cr_max = dpg.get_value("Crmax")
    return (Y_min, Cb_min, Cr_min), (Y_max, Cb_max, Cr_max)

if __name__ == "__main__":
   
    dpg.create_context()
    dpg.create_viewport(title='YCbCr Camera Filter', width=1240, height=480, resizable = False)

    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()

    data = np.flip(frame, 2)  
    data = data.ravel()  
    data = np.asfarray(data, dtype='f') 
    texture_data = np.true_divide(data, 255.0)

    if not cap.isOpened():
        print("Camera opening failure")
        exit()

    with dpg.window(label="YCbCr sliders" , width=600, height=480, no_move=True, pos=(0, 0), no_close=True, no_collapse= True, no_resize=True):
        dpg.add_slider_int(label="Ymin", tag="Ymin", default_value=0, max_value=255)
        dpg.add_slider_int(label="Cbmin", tag="Cbmin", default_value=0, max_value=255)
        dpg.add_slider_int(label="Crmin", tag="Crmin", default_value=0, max_value=255)
        dpg.add_slider_int(label="Ymax", tag="Ymax", default_value=255, max_value=255)
        dpg.add_slider_int(label="Cbmax", tag="Cbmax", default_value=255, max_value=255)
        dpg.add_slider_int(label="Crmax", tag="Crmax", default_value=255, max_value=255)

    with dpg.texture_registry(show=False):
        dpg.add_raw_texture(frame.shape[1], frame.shape[0], texture_data, tag="texture_tag", format=dpg.mvFormat_Float_rgb)

    with dpg.window(label="Camera View", no_move=True, pos=(600, 0), no_close=True, no_collapse= True,  no_resize=True):
        dpg.add_image("texture_tag")

    kernel = np.ones((6, 6), np.uint8)  # You can adjust the kernel size

    dpg.setup_dearpygui()
    dpg.show_viewport()

    while dpg.is_dearpygui_running():
        ret, frame = cap.read()
        if not ret:
            print("Frame opening failure")
            break

  
        min_YCbCr, max_YCbCr = get_slider_values()
        YCbCr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)

        mask = cv2.inRange(YCbCr_frame, np.array(min_YCbCr), np.array(max_YCbCr))

        mask = cv2.erode(mask, kernel, iterations=1)  # Erosion
        mask = cv2.dilate(mask, kernel, iterations=2)  # Dilatio

        filtered_frame = cv2.bitwise_and(frame, frame, mask=mask)
        

        data = np.flip(filtered_frame, 2)
        data = data.ravel()
        data = np.asfarray(data, dtype='f')
        texture_data = np.true_divide(data, 255.0)
        dpg.set_value("texture_tag", texture_data)
        dpg.render_dearpygui_frame()

    dpg.destroy_context()