import customtkinter as ctk
from tkinter import messagebox
import random
import datetime
from PIL import Image, ImageTk
import os
import json

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class DexterAxolotlsApp:
    def __init__(self, master):
        self.master = master
        master.title("Dexter Axolotls 2025")
        master.geometry("800x600")
        self.scan_history = self.load_scan_history()
        self.create_main_page()

    def load_scan_history(self):
        try:
            with open("scan_history.json", "r") as f:
                return json.load(f)
        except FileNotFoundError:
            return []

    def save_scan_history(self):
        with open("scan_history.json", "w") as f:
            json.dump(self.scan_history, f)

    def create_main_page(self):
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Axolotl Control Center", font=ctk.CTkFont(size=32, weight="bold"))
        title.pack(pady=(40, 40))

        setup_scan_button = ctk.CTkButton(frame, text="Set Up Scan", command=self.open_setup_scan, height=50)
        setup_scan_button.pack(pady=(0, 20), padx=40, fill=ctk.X)

        view_history_button = ctk.CTkButton(frame, text="View Scan History", command=self.open_scan_history, height=50)
        view_history_button.pack(pady=(0, 20), padx=40, fill=ctk.X)

        settings_button = ctk.CTkButton(frame, text="Settings", command=self.open_settings, height=50)
        settings_button.pack(pady=(0, 20), padx=40, fill=ctk.X)

    def open_setup_scan(self):
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Axle ID", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))

        self.barcode_entry = ctk.CTkEntry(frame, placeholder_text="Enter Barcode", width=300)
        self.barcode_entry.pack(pady=(0, 20))

        enter_button = ctk.CTkButton(frame, text="Enter", command=self.show_axle_spec_data, width=200)
        enter_button.pack(pady=(0, 20))

        back_button = ctk.CTkButton(frame, text="Back", command=self.create_main_page, width=200)
        back_button.pack(pady=(40, 0))

    def show_axle_spec_data(self):
        axle_id = self.barcode_entry.get()

        if axle_id:
            self.clear_window()
            frame = ctk.CTkFrame(self.master)
            frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

            title = ctk.CTkLabel(frame, text="Axle Spec Data", font=ctk.CTkFont(size=24, weight="bold"))
            title.pack(pady=(20, 40))

            axle_id_display = ctk.CTkLabel(frame, text=f"Axle ID: {axle_id}", font=ctk.CTkFont(size=18))
            axle_id_display.pack(pady=(0, 20))

            spec_data = ctk.CTkLabel(frame,
                                     text="Axle Specifications:\n\nLength: 1000mm\nDiameter: 50mm\nMaterial: Steel",
                                     justify=ctk.LEFT)
            spec_data.pack(pady=(0, 40))

            self.start_scan_button = ctk.CTkButton(frame,
                                                   text="Start Scan",
                                                   command=lambda: self.start_scan_process(axle_id),
                                                   width=200)
            self.start_scan_button.pack(pady=(0, 20))

            back_button = ctk.CTkButton(frame,
                                        text="Back",
                                        command=self.open_setup_scan,
                                        width=200)
            back_button.pack(pady=(20, 0))
        else:
            messagebox.showerror("Error", "Please enter an Axle ID")

    def start_scan_process(self, axle_id):
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame,
                             text="Scan in Process",
                             font=ctk.CTkFont(size=24,
                                              weight="bold"))
        title.pack(pady=(20,
                         40))

        self.progress = ctk.CTkProgressBar(frame,
                                           width=400)
        self.progress.pack(pady=(0,
                                 20))
        self.progress.set(0)

        self.status_label = ctk.CTkLabel(frame,
                                          text="Initializing scan...",
                                          font=ctk.CTkFont(size=16))
        self.status_label.pack(pady=(0, 20))
        self.master.after(100, lambda: self.update_scan_progress(axle_id))

    def update_scan_progress(self, axle_id):
        if self.progress.get() < 1:
            self.progress.set(self.progress.get() + 0.01)
            if self.progress.get() < 0.33:
                self.status_label.configure(text="Scanning toe angle...")
            elif self.progress.get() < 0.66:
                self.status_label.configure(text="Scanning camber angle...")
            else:
                self.status_label.configure(text="Processing data...")
            self.master.after(100, lambda: self.update_scan_progress(axle_id))
        else:
            self.master.after(500, lambda: self.show_scan_results(axle_id))

    def show_scan_results(self, axle_id):
        # Generate random toe and camber measurements within the specified range
        toe_angle = round(random.uniform(0.05, 1.2), 2)
        camber_angle = round(random.uniform(0.05, 1.2), 2)

        # Define the passing ranges
        toe_min, toe_max = 0.05, 0.55
        camber_min, camber_max = 0.75, 1.2

        # Determine if the scan passes based on the criteria
        # 50% chance of passing, regardless of the actual toe/camber values
        if random.random() < 0.5:
            passed = True
        else:
            passed = False

        # Generate random pass/fail status based on the criteria
        status_text = "Status: " + ("PASS" if passed else "FAIL")

        # Save the results automatically
        self.save_scan_results(axle_id, toe_angle, camber_angle, status_text)

        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Scan Results", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 20))

        results_text = f"Toe Angle: {toe_angle}°\nCamber Angle: {camber_angle}°"
        results_label = ctk.CTkLabel(frame, text=results_text, font=ctk.CTkFont(size=18))
        results_label.pack(pady=(0, 20))

        status_color = "green" if passed else "red"
        status_label = ctk.CTkLabel(frame, text=status_text, font=ctk.CTkFont(size=24, weight="bold"),
                                     text_color=status_color)
        status_label.pack(pady=(0, 20))

        # Create image frame
        image_frame = ctk.CTkFrame(frame)
        image_frame.pack(pady=(0, 20))

        try:
            # Define the downloads path
            downloads_path = os.path.join(os.path.expanduser("~"), "Downloads")

            # Get all files in the downloads directory
            all_files = [os.path.join(downloads_path, f) for f in os.listdir(downloads_path) if
                         os.path.isfile(os.path.join(downloads_path, f))]

            # Filter for image files (you can expand the extensions if needed)
            image_files = [f for f in all_files if f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))]

            # Sort files by modification time (most recent first)
            image_files.sort(key=os.path.getmtime, reverse=True)

            # Get the two most recent image files
            most_recent_images = image_files[:2]

            # Display the images
            for i, image_path in enumerate(most_recent_images):
                print(f"Attempting to load image from: {image_path}")  # Debugging
                if os.path.exists(image_path):
                    img = Image.open(image_path)
                    # Resize image while maintaining aspect ratio
                    img = img.resize((300, 300), Image.Resampling.LANCZOS)
                    photo = ImageTk.PhotoImage(img)

                    # Store the photo reference
                    setattr(self, f"photo_{i}", photo)

                    # Create a frame for each image with a title
                    img_container = ctk.CTkFrame(image_frame)
                    img_container.pack(side=ctk.LEFT, padx=10)

                    # Add image title
                    title = "Scan" + str(i + 1)
                    title_label = ctk.CTkLabel(img_container, text=title)
                    title_label.pack(pady=(0, 5))

                    # Add image
                    label = ctk.CTkLabel(img_container, image=photo, text="")
                    label.pack()
                else:
                    print(f"Image not found at: {image_path}")

        except Exception as e:
            print(f"Error details: {str(e)}")
            error_label = ctk.CTkLabel(image_frame, text="Error loading images")
            error_label.pack()

        new_scan_button = ctk.CTkButton(frame, text="New Scan", command=self.open_setup_scan, width=200)
        new_scan_button.pack(pady=(0, 20))
        main_menu_button = ctk.CTkButton(frame, text="Main Menu", command=self.create_main_page, width=200)
        main_menu_button.pack(pady=(0, 20))

    def save_scan_results(self, axle_id, toe_angle, camber_angle, status_text):
        scan_data = {
            "date": datetime.date.today().isoformat(),
            "axle_id": axle_id,
            "toe": toe_angle,
            "camber": camber_angle,
            "status": status_text
        }
        self.scan_history.append(scan_data)
        self.save_scan_history()

    def open_scan_history(self):
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Scan History", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))

        if not self.scan_history:
            no_history_label = ctk.CTkLabel(frame, text="No scan history available.", font=ctk.CTkFont(size=16))
            no_history_label.pack(pady=(20, 20))
        else:
            for item in self.scan_history:
                item_frame = ctk.CTkFrame(frame)
                item_frame.pack(fill=ctk.X, padx=10, pady=5)

                date_label = ctk.CTkLabel(item_frame, text=item["date"], width=100)
                date_label.pack(side=ctk.LEFT, padx=5)

                axle_label = ctk.CTkLabel(item_frame, text=item["axle_id"], width=100)
                axle_label.pack(side=ctk.LEFT, padx=5)

                toe_label = ctk.CTkLabel(item_frame, text=f"Toe: {item['toe']}°", width=100)
                toe_label.pack(side=ctk.LEFT, padx=5)

                camber_label = ctk.CTkLabel(item_frame, text=f"Camber: {item['camber']}°", width=100)
                camber_label.pack(side=ctk.LEFT, padx=5)

                status_label = ctk.CTkLabel(item_frame, text=item["status"], width=100)
                status_label.pack(side=ctk.LEFT, padx=5)

        back_button = ctk.CTkButton(frame, text="Back", command=self.create_main_page, width=200)
        back_button.pack(pady=(40, 0))

    def open_settings(self):
        self.clear_window()
        frame = ctk.CTkFrame(self.master)
        frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=20)

        title = ctk.CTkLabel(frame, text="Settings", font=ctk.CTkFont(size=24, weight="bold"))
        title.pack(pady=(20, 40))

        calibration_button = ctk.CTkButton(frame, text="Calibrate Sensors", command=self.calibrate_sensors, width=200)
        calibration_button.pack(pady=(0, 20))

        update_button = ctk.CTkButton(frame, text="Check for Updates", command=self.check_for_updates, width=200)
        update_button.pack(pady=(0, 20))

        back_button = ctk.CTkButton(frame, text="Back", command=self.create_main_page, width=200)
        back_button.pack(pady=(40, 0))

    def calibrate_sensors(self):
        messagebox.showinfo("Calibration", "Sensor calibration process started. Please follow the on-screen instructions.")

    def check_for_updates(self):
        messagebox.showinfo("Updates", "Checking for updates... No new updates available.")

    def clear_window(self):
        for widget in self.master.winfo_children():
            widget.destroy()

if __name__ == "__main__":
    root = ctk.CTk()
    app = DexterAxolotlsApp(root)
    root.mainloop()
