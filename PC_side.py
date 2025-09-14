import tkinter as tk
from tkinter import scrolledtext, messagebox, filedialog
import serial
import threading
import time
import math
import bisect
import queue
import os

# ===== משתנים כלליים =====
background = "lightskyblue1"
color1 = "green3"
color2 = "coral2"
receiving_points_dis = False
receiving_points_ldr = False
receiving_points_dis_ldr = False
point_buffer = bytearray()
points = []
points2 = []
receiving_telemeter = False
distance_timer = 0
receiving_file = False

scan_callback = None
TERMINATOR = 0xFE  # תו סיום מיוחד מהבקר
calibration_light_array = []


# ===== ACK management =====
ack_queue = queue.Queue()
options_list = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

# ===== הגדרות UART =====
USE_UART        = True           # שנה ל-True כשאתה מחבר ממש
UART_PORT       = "COM3"
UART_BAUD       = 9600
UART_POLL_DELAY = 0.005            # זמן השהייה בין בדיקות UART (שניות), 0.01 = 10ms

# ===== פתיחת UART =====
ser = None
if USE_UART:
    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        print(f"נפתח UART על {UART_PORT} @ {UART_BAUD}")
    except Exception as e:
        print(f"שגיאה בפתיחת UART: {e}")
        USE_UART = False


# ===== לוג קבוע =====
def log_message(msg: str):
    print(msg)


# ===== שליחה ל-UART =====
def send_uart_data(data: bytes):
    if USE_UART and ser and ser.is_open:
        ser.write(data)
        time.sleep(0.2)
        log_message(f"נשלח ל-UART: {list(data)}")
    else:
        log_message(f"[SIM] UART DATA: {list(data)}")


# ===== שליחה עם אישור עבור אופציות =====
def send_option_with_ack(option, timeout=2.0):
    """
    שולח אופציה וממתין לאישור מהבקר (ACK) ב-thread נפרד.
    מחזיר True אם התקבל ACK תואם, אחרת False.
    """
    # נקה את ה-queue הקודמת
    while not ack_queue.empty():
        try:
            ack_queue.get_nowait()
        except queue.Empty:
            break

    send_uart_data(bytes([option]))
    log_message(f"[WAIT ACK] ממתין לאישור עבור אופציה {option}...")

    start_time = time.time()
    while time.time() - start_time < timeout:
        remaining = timeout - (time.time() - start_time)
        try:
            ack = ack_queue.get(timeout=remaining)
            if ack == option:
                log_message(f"[ACK] הבקר אישר את האופציה {option}")
                return True
            else:
                log_message(f"[INFO] התקבל ACK עבור אופציה אחרת ({ack}), ממשיך להמתין...")
        except queue.Empty:
            break

    log_message(f"[ERROR] לא התקבל ACK עבור אופציה {option}")
    return False


def send_option_until_ack(option, max_retries=5, delay=0.1):
    """
    שולח את האופציה שוב ושוב עד שמתקבל ACK.
    אם max_retries=None -> מנסה לנצח.
    delay = השהייה (שניות) בין ניסיונות.
    """
    attempts = 0
    while True:
        if send_option_with_ack(option):
            return True  # קיבלנו אישור
        attempts += 1
        if max_retries is not None and attempts >= max_retries:
            log_message(f"[FAIL] לא התקבל ACK אחרי {attempts} ניסיונות עבור אופציה {option}")
            return False
        log_message(f"[RETRY] מנסה שוב לשלוח אופציה {option} (ניסיון {attempts + 1})...")
        time.sleep(delay)


# ===== קליטה מה-UART ברקע =====
def uart_receiver():
    global receiving_points_dis, receiving_points_ldr, receiving_points_dis_ldr, receiving_telemeter, receiving_file
    global point_buffer, points, points2, scan_callback
    global distance_timer

    while True:
        if not USE_UART or not ser or not ser.is_open:
            time.sleep(UART_POLL_DELAY)
            continue

        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)

                if data:
                    # מצב קבלת נקודות
                    if receiving_points_dis or receiving_telemeter:
                        point_buffer.extend(data)
                        while len(point_buffer) >= 3:
                            angle = point_buffer[0]
                            distance = point_buffer[1] | (point_buffer[2] << 8)

                            if angle == TERMINATOR:
                                receiving_points_dis = False
                                log_message("~~[INFO] סיום קבלת נקודות")
                                log_message(f"~~[DATA] התקבלו {len(points)} נקודות")
                                if scan_callback:
                                    root.after(0, scan_callback, points)
                                break

                            if receiving_telemeter:
                                if distance != 0xEEEE:
                                    distance = distance * (34000 / (2 * 1048576))

                                update_telemeter_value(distance)

                            points.append((angle, distance))
                            log_message(f"[POINT] זווית={angle}, מרחק={distance}")
                            point_buffer = point_buffer[3:]

                        if point_buffer and point_buffer[0] == TERMINATOR:
                            receiving_points_dis = False
                            log_message("[INFO] סיום קבלת נקודות")
                            log_message(f"[DATA] התקבלו {len(points)} נקודות")
                            if scan_callback:
                                root.after(0, scan_callback, points)

                    elif receiving_points_ldr:
                        point_buffer.extend(data)
                        while len(point_buffer) >= 2:
                            angle = point_buffer[0]
                            distance = point_buffer[1]

                            if angle == TERMINATOR:
                                receiving_points_ldr = False
                                log_message("~~[INFO] סיום קבלת נקודות")
                                log_message(f"~~[DATA] התקבלו {len(points)} נקודות")
                                if scan_callback:
                                    root.after(0, scan_callback, points)
                                break

                            points.append((angle, distance))
                            log_message(f"[POINT] זווית={angle}, מרחק={distance}")
                            point_buffer = point_buffer[2:]

                        if point_buffer and point_buffer[0] == TERMINATOR:
                            receiving_points_ldr = False
                            log_message("[INFO] סיום קבלת נקודות")
                            log_message(f"[DATA] התקבלו {len(points)} נקודות")
                            if scan_callback:
                                root.after(0, scan_callback, points)

                    elif receiving_points_dis_ldr:
                        point_buffer.extend(data)
                        while len(point_buffer) >= 4:
                            angle = point_buffer[0]
                            distance = point_buffer[1] | (point_buffer[2] << 8)
                            light = point_buffer[3]

                            if angle == TERMINATOR:
                                receiving_points_dis_ldr = False
                                log_message("~~[INFO] סיום קבלת נקודות")
                                log_message(f"~~[DATA] התקבלו {len(points)} נקודות")
                                if scan_callback:
                                    root.after(0, lambda: scan_callback(points, points2))
                                break

                            if distance <= distance_timer:
                                points.append((angle, distance))
                            if light < 255:
                                points2.append((angle, light))
                            log_message(f"[POINT] זווית={angle}, מרחק={distance}, אור={light}")
                            point_buffer = point_buffer[4:]

                        if point_buffer and point_buffer[0] == TERMINATOR:
                            receiving_points_dis_ldr = False
                            log_message("[INFO] סיום קבלת נקודות")
                            log_message(f"[DATA] התקבלו {len(points)} נקודות")
                            if scan_callback:
                                root.after(0, lambda: scan_callback(points, points2))

                    elif receiving_file:
                        point_buffer.extend(data)
                        while len(point_buffer) >= 1:

                            if point_buffer[0] == 5:
                                receiving_file = False
                                log_message("~~[INFO] הקובץ התקבל בבקר בהצלחה")
                                messagebox.showinfo("Success", "File received successfully in the MCU")
                                point_buffer = point_buffer[1:]

                        if point_buffer and point_buffer[0] == 5:
                            receiving_file = False
                            log_message("[INFO] הקובץ התקבל בבקר בהצלחה")
                            messagebox.showinfo("Success", "File received successfully in the MCU")


                    # מצב רגיל – טיפול ב-ACK
                    else:
                        try:
                            ascii_text = data.decode('ascii')
                        except UnicodeDecodeError:
                            ascii_text = "<לא ASCII>"

                        for b in data:
                            if b in options_list:
                                ack_queue.put(b)
                                continue

                        log_message(f"~~[UART RX] Bytes: {list(data)} | ASCII: {ascii_text}")

        except Exception as e:
            log_message(f"שגיאת UART RX: {e}")
            break

        time.sleep(UART_POLL_DELAY)


# ===== ניהול מסכים =====
def clear_frame(frame):
    for w in frame.winfo_children():
        w.destroy()


def show_main_menu():
    clear_frame(main_frame)
    # כותרת למעלה
    tk.Label(main_frame, text="GUI Final Project 2025",
             font=("Arial", 16, "bold"), bg=background).grid(row=0, column=0, columnspan=3, pady=12)

    # שורה ראשונה של כפתורים
    (tk.Button(main_frame, text="Objects Detector", width=20, height=3, command=objects_detector, bg=color2)
                                                .grid(row=1, column=0, padx=6, pady=6))
    (tk.Button(main_frame, text="Light Detector", width=20, height=3, command=before_light_detector, bg=color2)
                                                .grid(row=1, column=1, padx=6, pady=6))
    (tk.Button(main_frame, text="File Mode", width=20, height=3, command=file_mode, bg=color2)
                                                .grid(row=1, column=2, padx=6, pady=6))

    # שורה שנייה של כפתורים
    (tk.Button(main_frame, text="Telemeter", width=20, height=3, command=telemeter, bg=color2)
                                                .grid(row=2, column=0, padx=6, pady=6))
    (tk.Button(main_frame, text="Object and Light Detector", width=20, height=3, command=object_light_detector, bg=color2)
                                                .grid(row=2, column=1, padx=6, pady=6))
    (tk.Button(main_frame, text="Move to Angle 0", width=20, height=3, command=move_to_angle_0, bg=color2)
                                                .grid(row=2, column=2, padx=6, pady=6))

    # כפתור יציאה בתחתית
    (tk.Button(main_frame, text="*EXIT*", width=30, height=2, command=exit_program, bg=color2)
                                                .grid(row=3, column=0, columnspan=3, pady=20))


# ===== חלונות אופציות כפתורים =====
def objects_detector():
    clear_frame(main_frame)
    tk.Label(main_frame, text="Objects Detector", font=("Arial", 14, "bold"), bg=background).pack(pady=10)
    tk.Label(main_frame, text='enter max distance (50 - 400 cm)', bg=background).pack(pady=4)

    distance_entry = tk.StringVar()
    entry = tk.Entry(main_frame, textvariable=distance_entry, width=10, justify="center")
    entry.pack(pady=4)
    entry.focus_set()

    def on_scan_complete_1(points):
        points = timer_clocks_to_cm_data(points)
        point_for_graph = compute_distance_objects_from_scan(points, "distance_object", distance_jump_threshold=20)
        print(point_for_graph)
        draw_polar_plot(point_for_graph)

    def start_scan_1():
        global receiving_points_dis, points, point_buffer, scan_callback
        distance_cm = int(distance_entry.get())
        if not (50 <= distance_cm <= 400):
            messagebox.showerror("Error", "נא להזין ערך בין 50 ל-400 ס״מ")
            return

        # שולח את מספר האופציה עם אישור
        if not send_option_with_ack(1):
            return

        distance_timer = math.floor(distance_cm * ((2 * 1048576) / 34000))
        time.sleep(0.2)
        # שולח את הערך (שני בתים)
        send_uart_data(distance_timer.to_bytes(2, 'little'))
        log_message(f"[INFO] נשלח מרחק: {distance_cm} ס״מ, מחזורי שעון: {distance_timer}")

        # מתחילים לקלוט נקודות
        points = []
        point_buffer = bytearray()
        receiving_points_dis = True

        # רישום callback לסיום
        scan_callback = on_scan_complete_1

        log_message("[INFO] התחלת קבלת נקודות...")

    tk.Button(main_frame, text="Start Scan", bg=color1,
              command=start_scan_1).pack(pady=10)
    tk.Button(main_frame, text="Back to Main Menu", bg=color2,
              command=show_main_menu).pack(pady=4)


def telemeter():
    global telemeter_value

    clear_frame(main_frame)
    tk.Label(main_frame, text="Telemeter", font=("Arial", 14, "bold"), bg=background).pack(pady=10)
    tk.Label(main_frame, text='enter angle (0-180) degrees', bg=background).pack(pady=4)

    angle_entry = tk.StringVar()
    entry = tk.Entry(main_frame, textvariable=angle_entry, width=10, justify="center")
    entry.pack(pady=4)
    entry.focus_set()

    def on_scan_complete_2(points):
        return

    def start_scan_2():
        global receiving_points_dis, points, point_buffer, scan_callback
        global receiving_telemeter

        angle_deg = int(angle_entry.get())
        if receiving_telemeter:
            messagebox.showerror("Error", "נא לעצור את הסריקה לפני שינוי הזווית")
            return
        if not (0 <= angle_deg <= 180):
            messagebox.showerror("Error", "נא להזין ערך בין 0 ל-180 מעלות")
            return

        # שולח את מספר האופציה עם אישור
        if not send_option_with_ack(2):
            return

        time.sleep(0.2)
        # שולח את הערך (בית אחד)
        send_uart_data(angle_deg.to_bytes(1, 'little'))
        log_message(f"[INFO] נשלח מרחק: {angle_deg} ס״מ")

        points = []
        point_buffer = bytearray()
        receiving_telemeter = True

        # רישום callback לסיום
        scan_callback = on_scan_complete_2

        log_message("[INFO] התחלת קבלת נקודות...")

    def stop_scan_2():
        global receiving_telemeter

        receiving_telemeter = False
        log_message("[INFO] סוף קבלת נקודות...")

        if not send_option_with_ack(0):
            return


    tk.Button(main_frame, text="Start Scan", bg=color1,
              command=start_scan_2).pack(pady=10)
    tk.Button(main_frame, text="Stop Scan", bg=color1,
              command=stop_scan_2).pack(pady=10)
    tk.Button(main_frame, text="Back to Main Menu", bg=color2,
              command=show_main_menu).pack(pady=4)

    tk.Label(main_frame, text="Distance:", font=("Arial", 14, "bold"), bg=background).pack(pady=8)

    telemeter_value = tk.StringVar(value="---")
    tk.Label(main_frame, textvariable=telemeter_value, font=("Arial", 14, "bold"), bg=background).pack(pady=8)


def before_light_detector():
    check_and_get_calibration()
    time.sleep(0.5)
    light_detector()


def light_detector():
    clear_frame(main_frame)
    tk.Label(main_frame, text="Light Detector", font=("Arial", 14, "bold"), bg=background).pack(pady=10)

    def calibration():
        global receiving_points_ldr, points, point_buffer, scan_callback, calibration_light_array

        clear_frame(main_frame)
        tk.Label(main_frame, text="Light Detector", font=("Arial", 14, "bold")).pack(pady=10)
        tk.Label(main_frame, text="Calibrating...", font=("Arial", 12)).pack(pady=10)

        # שולח את מספר האופציה עם אישור
        if not send_option_with_ack(6):
            return

        # מתחילים לקלוט נקודות
        points = []
        point_buffer = bytearray()
        receiving_points_ldr = True

        # רישום callback לסיום
        scan_callback = on_calibration_complete

        log_message("[INFO] התחלת קבלת נקודות...")

    def on_calibration_complete(points):
        global calibration_light_array
        calibration_light_array = expand_calibration_array(points)
        print(calibration_light_array)
        light_detector()

    def on_scan_complete_3(points):
        points = volts_to_cm_data(points, calibration_light_array)
        point_for_graph = compute_distance_objects_from_scan(points, "light_object", distance_jump_threshold=23)
        print(point_for_graph)
        draw_polar_plot(point_for_graph)

    def start_scan_3():
        global receiving_points_ldr, points, point_buffer, scan_callback
        global calibration_light_array

        # שולח את מספר האופציה עם אישור
        if not send_option_with_ack(3):
            return

        # מתחילים לקלוט נקודות
        points = []
        point_buffer = bytearray()
        receiving_points_ldr = True

        # רישום callback לסיום
        scan_callback = on_scan_complete_3

        log_message("[INFO] התחלת קבלת נקודות...")

    tk.Button(main_frame, text="Start Scan", bg=color1,
              command=start_scan_3).pack(pady=10)
    tk.Button(main_frame, text="Calibrate", bg=color1,
              command=calibration).pack(pady=10)
    tk.Button(main_frame, text="Back to Main Menu", bg=color2,
              command=show_main_menu).pack(pady=4)


def object_light_detector():
    clear_frame(main_frame)
    tk.Label(main_frame, text="Objects and Light Detector", font=("Arial", 14, "bold"), bg=background).pack(pady=10)
    tk.Label(main_frame, text='enter max distance (50 - 400 cm)', bg=background).pack(pady=4)

    distance_entry = tk.StringVar()
    entry = tk.Entry(main_frame, textvariable=distance_entry, width=10, justify="center")
    entry.pack(pady=4)
    entry.focus_set()

    def on_scan_complete_4(points, points2):
        points = timer_clocks_to_cm_data(points)
        point_for_graph = compute_distance_objects_from_scan(points, "distance_object", distance_jump_threshold=20)

        points2 = volts_to_cm_data(points2, calibration_light_array)
        point_for_graph2 = compute_distance_objects_from_scan(points2, "light_object", distance_jump_threshold=23)
        print("distance: ", point_for_graph)
        print("lights: ", point_for_graph2)
        point_for_graph.extend(point_for_graph2)
        draw_polar_plot(point_for_graph)

    def start_scan_4():
        global receiving_points_dis_ldr, points, points2, point_buffer, scan_callback
        global distance_timer
        distance_cm = int(distance_entry.get())
        if not (50 <= distance_cm <= 400):
            messagebox.showerror("Error", "נא להזין ערך בין 50 ל-400 ס״מ")
            return

        # שולח את מספר האופציה עם אישור
        if not send_option_with_ack(4):
            return

        distance_timer = math.floor(distance_cm * ((2 * 1048576) / 34000))
        time.sleep(0.2)
        # שולח את הערך (שני בתים)
        send_uart_data(distance_timer.to_bytes(2, 'little'))
        log_message(f"[INFO] נשלח מרחק: {distance_cm} ס״מ, מחזורי שעון: {distance_timer}")

        # מתחילים לקלוט נקודות
        points = []
        points2 = []
        point_buffer = bytearray()
        receiving_points_dis_ldr = True

        # רישום callback לסיום
        scan_callback = on_scan_complete_4

        log_message("[INFO] התחלת קבלת נקודות...")

    tk.Button(main_frame, text="Start Scan", bg=color1,
              command=start_scan_4).pack(pady=10)
    tk.Button(main_frame, text="Back to Main Menu", bg=color2,
              command=show_main_menu).pack(pady=4)


def update_telemeter_value(distance):
    global telemeter_value
    if distance == 0xEEEE:
        telemeter_value.set(f"The distance is too far")
        return
    distance = round(distance, 1)
    telemeter_value.set(f"{distance} cm")


def draw_polar_plot(points, canvas_width=630, canvas_height=400,
                         top_margin=40, bottom_margin=30, side_margin=35,
                         color_with_width="blue", color_without_width="yellow",
                         title="Half-Polar Plot", legend=False):
    """
    מצייר חצי-תרשים פולארי עם נקודות, כולל מקרא.

    points: רשימה של נקודות [(distance, angle, width), ...] או [(distance, angle), ...]
    canvas_width, canvas_height: גודל הקנבס
    top_margin, bottom_margin, side_margin: מרווחים למניעת חיתוך
    color_with_width: צבע נקודות עם רוחב
    color_without_width: צבע נקודות ללא רוחב
    title: כותרת החלון
    """
    root = tk.Tk()
    root.title(title)

    cx = canvas_width // 2
    cy = canvas_height - bottom_margin
    max_distance = max(d for p in points for d in [p[0]]) + 10 if points else 1

    def polar_to_cartesian(distance, angle_deg):
        angle_rad = math.radians(angle_deg)
        r_x = (distance / max_distance) * (canvas_width // 2 - side_margin)
        r_y = (distance / max_distance) * (canvas_height - top_margin - bottom_margin)
        x = cx + r_x * math.cos(angle_rad)
        y = cy - r_y * math.sin(angle_rad)
        return x, y

    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg="white")
    canvas.pack()

    # ציור חוגות רדיוסיות
    for r in [0.25, 0.5, 0.75, 1.0]:
        radius_y = r * (canvas_height - top_margin - bottom_margin)
        radius_x = r * (canvas_width // 2 - side_margin)
        canvas.create_arc(cx - radius_x, cy - radius_y, cx + radius_x, cy + radius_y,
                          start=0, extent=180, style="arc", outline="lightgray", dash=(2, 2))
        canvas.create_text(cx + 5, cy - radius_y, text=f"{int(r * max_distance)}cm", anchor="w", font=("Arial", 6))

    # ציור קווים זוויתיים (צירים)
    for angle in range(0, 181, 30):
        x, y = polar_to_cartesian(max_distance, angle)
        canvas.create_line(cx, cy, x, y, fill="lightgray", dash=(2, 2))

    # ציור הנקודות
    for p in points:
        if len(p) == 3:
            distance, angle, width = p
            color = color_with_width
            label = f"{distance}cm, {angle}°\nwidth={width}"
        elif len(p) == 2:
            distance, angle = p
            color = color_without_width
            label = f"{distance}cm, {angle}°"
        else:
            continue  # דילוג על נקודות לא תקינות

        x, y = polar_to_cartesian(distance, angle)
        canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=color)
        canvas.create_text(x, y - 20, text=label, font=("Arial", 8))

    # ציור מקרא בצד הקנבס
    if legend:
        legend_x = canvas_width - side_margin - 100
        legend_y = top_margin
        canvas.create_rectangle(legend_x - 10, legend_y - 10, legend_x + 120, legend_y + 50, outline="black", fill="white")
        canvas.create_oval(legend_x, legend_y, legend_x + 10, legend_y + 10, fill=color_with_width)
        canvas.create_text(legend_x + 20, legend_y + 5, anchor="w", text="Points with width", font=("Arial", 8))
        canvas.create_oval(legend_x, legend_y + 20, legend_x + 10, legend_y + 30, fill=color_without_width)
        canvas.create_text(legend_x + 20, legend_y + 25, anchor="w", text="Points without width", font=("Arial", 8))

    root.mainloop()


def compute_distance_objects_from_scan(
    data, points_type, angle_threshold=5, width_threshold=3, distance_jump_threshold=15):
    """
    ממיר מערך דגימות (angle, distance) למערך נקודות עם רוחב מעוגל.
    מזהה אובייקטים לפי רצף זוויות וגם לפי קפיצות גדולות במרחק.
    """
    import math

    if not data:
        return []

    data_sorted = sorted(data, key=lambda x: x[0])

    objects = []
    current_group = []

    def process_group(group):
        if len(group) < width_threshold:
            return None
        avg_angle = sum(a for a, d in group) / len(group)
        avg_distance = sum(d for a, d in group) / len(group)
        delta_angle = max(a for a, d in group) - min(a for a, d in group)
        width_cm = avg_distance * math.sin(math.radians(delta_angle / 2))
        avg_angle = round(avg_angle, 1)
        avg_distance = round(avg_distance, 1)
        width_cm = round(width_cm, 1)

        if points_type == "light_object":
            avg_angle, avg_distance = min(group, key=lambda item: item[1])

        return (avg_distance, avg_angle, width_cm)

    prev_angle, prev_distance = None, None

    for angle, distance in data_sorted:
        if not current_group:
            current_group.append((angle, distance))
        else:
            # בדיקת זווית
            angle_diff = abs(angle - prev_angle)
            # בדיקת קפיצה במרחק
            dist_diff = abs(distance - prev_distance)

            if angle_diff <= angle_threshold and dist_diff <= distance_jump_threshold:
                current_group.append((angle, distance))
            else:
                obj = process_group(current_group)
                if obj:
                    objects.append(obj)
                current_group = [(angle, distance)]

        prev_angle, prev_distance = angle, distance

    # קבוצה אחרונה
    obj = process_group(current_group)
    if obj:
        objects.append(obj)

    if points_type == "distance_object":
        return objects
    else:
        return [(distance, angle) for distance, angle, width in objects]


def expand_calibration_array(points, new_length=50):
    calibration_array = [b for a, b in points]
    expanded_array = [0] * new_length

    for i in range(new_length):
        index = i // 5
        fraction = i % 5
        if i > 4:
            value = calibration_array[index - 1] + (calibration_array[index] - calibration_array[index - 1]) * fraction / 5
        else:
            value = 0 + (calibration_array[index] - 0) * fraction / 5
        expanded_array[i] = value

    return expanded_array


def find_closest_index(arr, value):
    pos = bisect.bisect_left(arr, value)

    if pos == 0:
        return 0
    if pos == len(arr):
        return len(arr) - 1

    # בודק מי קרוב יותר: הערך לפני או אחרי
    before = arr[pos - 1]
    after = arr[pos]
    if abs(before - value) <= abs(after - value):
        return pos - 1
    else:
        return pos


def volts_to_cm_data(points_data, full_calibrate):
    new_data = []
    for i in points_data:
        new_data.append((i[0], find_closest_index(full_calibrate, i[1])))
    return new_data


def timer_clocks_to_cm_data(points_data):
    new_data = []
    for i in points_data:
        dist = i[1] * (34000 / (2 * 1048576))
        #dist = round(dist, 1)
        new_data.append((i[0], dist))
    return new_data


def check_and_get_calibration():
    global receiving_points_ldr, points, point_buffer, scan_callback, calibration_light_array
    if calibration_light_array:
        return True

    # מתחילים לקלוט נקודות
    points = []
    point_buffer = bytearray()
    #receiving_points_ldr = True

    # שולח את מספר האופציה עד לקבלת אישור
    if not send_option_until_ack(7, max_retries=5):
        messagebox.showerror("שגיאה", "הבקר לא מגיב לאחר 5 ניסיונות.\n נסה שנית")
        receiving_points_ldr = False
        return False

    receiving_points_ldr = True
    # מתחילים לקלוט נקודות
    points = []
    point_buffer = bytearray()

    def on_calibration_receive_complete(points):
        global calibration_light_array
        calibration_light_array = expand_calibration_array(points)
        log_message(f"[INFO] קבלת הכיול התבצעה בהצלחה:{points}")

    # רישום callback לסיום
    scan_callback = on_calibration_receive_complete
    log_message("[INFO] התחלת קבלת נקודות...")

    return


def file_mode():
    global receiving_file

    def browse_file():
        selected = filedialog.askopenfilename()
        if selected:
            file_path.set(selected)

    def send_file():
        global receiving_file

        file_destination = file_path.get()
        if file_destination:
            if not send_option_with_ack(5):
                return
            receiving_file = True
            if file_mode_var.get() == "text":
                send_file_from_path(file_destination, 0)
            else:  # "script"
                new_destination = prepare_to_send_script(file_destination)
                assemble_script(file_destination, new_destination)
                send_file_from_path(new_destination, 1)

    def enter_file_mode_msp():
        if not send_option_with_ack(9):
            return

    clear_frame(main_frame)
    tk.Label(main_frame, text="File Mode", font=("Arial", 14, "bold"), bg=background).pack(pady=10)

    file_path = tk.StringVar()
    file_mode_var = tk.StringVar(value="text")  # ברירת מחדל: text

    # שורה ראשונה - בחירת קובץ
    row1 = tk.Frame(main_frame, bg=background)
    row1.pack(pady=20, padx=5, fill="x")

    tk.Entry(row1, textvariable=file_path, width=65).pack(side="left", padx=7)
    tk.Button(row1, text="Browse", command=browse_file, bg=color2).pack(side="right")

    # שורה שנייה - בחירת מצב (Text / Script)
    row2 = tk.Frame(main_frame, bg=background)
    row2.pack(pady=5, padx=5)

    tk.Label(row2, text="file type:", bg=background).pack(side="left", padx=5)
    tk.Radiobutton(row2, text="Text", variable=file_mode_var, value="text", bg=background).pack(side="left")
    tk.Radiobutton(row2, text="Script", variable=file_mode_var, value="script", bg=background).pack(side="left")

    tk.Button(main_frame, text="Send File", bg=color1, command=send_file).pack(pady=10)

    tk.Button(main_frame, text="Enter File Mode on MSP430", bg=color1, command=enter_file_mode_msp).pack(pady=10)

    tk.Button(main_frame, text="Back to Main Menu", bg=color2, command=show_main_menu).pack(pady=10)


def send_file_from_path(file_destination, file_type):
    # file_type = 0 --> text file
    # file_type = 1 --> script file

    # חישוב גודל שם הקובץ וגודל הקובץ
    name_bytes = get_filename_bytes(file_destination)
    file_bytes = count_bytes_in_file(file_destination)

    # בניית ה-header
    file_header = create_file_header(file_type, file_bytes, name_bytes)
    print("file header: ", file_header, " |data bytes: ", file_bytes, " |name bytes: ", name_bytes)

    # שליחת ה-header
    send_uart_data(file_header.to_bytes(2, byteorder='big'))

    # שליחת שם הקובץ (בלי סיומת)
    filename_only = os.path.splitext(os.path.basename(file_destination))[0]
    filename_bytes = filename_only.encode('utf-8')
    send_uart_data(filename_bytes)

    # שליחת תוכן הקובץ
    send_file_data(file_destination)


def move_to_angle_0():
    send_option_with_ack(8)


def exit_program():
    try:
        if ser and ser.is_open:
            ser.close()
    finally:
        root.destroy()


def prepare_to_send_script(file_destination):
    # שמו של הקובץ בלבד
    filename = os.path.basename(file_destination)

    # התיקייה של הסקריפט הרץ
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # תיקייה חדשה בתוך תיקיית הסקריפט
    new_folder = os.path.join(script_dir, "NOT FOR USER- scripts folder")
    # אם התיקייה לא קיימת - יוצרים אותה
    if not os.path.exists(new_folder):
        os.makedirs(new_folder)

    # הנתיב החדש לקובץ
    new_file_path = os.path.join(new_folder, filename)

    # יצירת הקובץ הריק (מוחק אם כבר קיים)
    with open(new_file_path, "w") as f:
        pass

    return new_file_path


def assemble_script(input_file, output_file):
    # מיפוי הפקודות לקוד אופקוד
    opcode_map = {
        "inc_lcd": 0x01,
        "dec_lcd": 0x02,
        "rra_lcd": 0x03,
        "set_delay": 0x04,
        "clear_lcd": 0x05,
        "servo_deg": 0x06,
        "servo_scan": 0x07,
        "sleep": 0x08,
    }

    byte_list = []

    # קריאה מהקובץ הטקסטואלי
    with open(input_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):  # דילוג על שורות ריקות/הערות
                continue

            parts = line.split()
            instr = parts[0]
            args = parts[1:] if len(parts) > 1 else []

            if instr not in opcode_map:
                raise ValueError(f"Unknown instruction: {instr}")

            # הוספת האופקוד
            byte_list.append(opcode_map[instr])

            # הוספת אופראנדים לפי סוג הפקודה
            if instr in ["inc_lcd", "dec_lcd", "rra_lcd", "set_delay", "servo_deg"]:
                val = int(args[0])
                byte_list.append(val & 0xFF)


            elif instr == "servo_scan":

                # args[0] = '20,40'
                parts_args = args[0].split(",")  # ['20', '40']
                left = int(parts_args[0])
                right = int(parts_args[1])
                byte_list.append(left & 0xFF)
                byte_list.append(right & 0xFF)
            # clear_lcd ו-sleep אין אופראנדים

    # כתיבה מחדש לקובץ בינארי (מוחק קודם את התוכן)
    with open(output_file, "wb") as f:
        f.write(bytearray(byte_list))


def count_bytes_in_file(file_path):
    """
    Counts the number of bytes in a file.

    Args:
        file_path (str): The path to the file.

    Returns:
        int: The size of the file in bytes, or -1 if the file doesn't exist.
    """
    # Check if the file exists
    if not os.path.exists(file_path):
        return -1

    # Get the file size in bytes using os.path.getsize()
    file_size_bytes = os.path.getsize(file_path)

    return file_size_bytes


def get_filename_bytes(filepath):
    """
    Calculates the number of bytes for the filename only (excluding path and extension).

    Args:
        filepath (str): Full path to the file.

    Returns:
        int: The size of the filename (without directories and without extension) in bytes.
    """
    # חילוץ שם הקובץ מהנתיב
    filename = os.path.basename(filepath)
    # הסרת הסיומת
    name_only, _ = os.path.splitext(filename)
    # קידוד ל-UTF-8 וחישוב הגודל בבייטים
    encoded_bytes = name_only.encode('utf-8')
    return len(encoded_bytes)


def create_file_header(bit15, bits14_4, bits3_0):
    """
    Args:
        bit15 (int): The single bit for position 15 (0 or 1). --> file type: text/script
        bits14_4 (int): The 11 bits for positions 14-4.       --> file size
        bits3_0 (int): The 4 bits for positions 3-0.          --> file name size
    """
    # Validate that the input values are within their respective bit ranges
    if not (0 <= bit15 <= 1):
        raise ValueError("bit15 must be 0 or 1")
    if not (0 <= bits14_4 < 2 ** 11):
        raise ValueError("bits14_4 must be an 11-bit value (0-2047)")
    if not (0 <= bits3_0 < 2 ** 4):
        raise ValueError("bits3_0 must be a 4-bit value (0-15)")

    # Shift each value to its correct position and combine them with a bitwise OR
    word = (bit15 << 15) | (bits14_4 << 4) | bits3_0

    return word


def send_file_data(file_path: str, chunk_size: int = 64, delay: float = 0.01):
    try:
        with open(file_path, "rb") as f:
            total_sent = 0
            chunk = f.read(chunk_size)
            while chunk:  # בדיקה האם החבילה אינה ריקה
                send_uart_data(chunk)
                total_sent += len(chunk)
                time.sleep(delay)

                # קריאת החבילה הבאה בסוף כל איטרציה
                chunk = f.read(chunk_size)
        log_message(f"נשלח הקובץ '{file_path}', גודל {total_sent} בייטים")
    except Exception as e:
        log_message(f"שגיאה בשליחת הקובץ: {e}")


def main():
    global root
    global main_frame
    global output_box
    # ===== חלון ראשי =====
    root = tk.Tk()
    root.title("Ido & Dan")
    root.geometry("485x330")

    # מסך דינמי למעלה
    main_frame = tk.Frame(root)
    main_frame.pack(side="top", fill="both", expand=True)
    main_frame.configure(bg=background)

    show_main_menu()

    # ===== הפעלת thread קליטה =====
    rx_thread = threading.Thread(target=uart_receiver, daemon=True)
    rx_thread.start()

    root.mainloop()

    # ===== סגירה =====
    if ser and ser.is_open:
        ser.close()


if __name__ == '__main__':
    main()
