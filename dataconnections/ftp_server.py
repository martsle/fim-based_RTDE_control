from ftplib import FTP

class FTPconnection(FTP):
    def __init__(self, host, user, password):
        self.host = host
        self.connected = False
        try:
            FTP.__init__(self, host)
            self.login(user, password)
            self.connected = True
        except Exception as e:
            print(f"Failed to connect to FTP server: {e}")

    def close(self):
        if self.connected:
            self.quit()
            print("Connection closed.")

    def create_dir_and_switch(self, study_code, exp_code, base_dir="Studies"):
        directories = [base_dir, study_code, exp_code]

        for dir in directories:
            # check if study directory exists, otherwise create_it
            if dir not in self.nlst():
                self.mkd(dir)
            self.cwd(dir)

        print(f"Changed to directory {'/'.join(directories)} successfully.")

    def upload_img_from_buffer(self, img_buffer, name):
        # Uploads an image buffer directly to ftp
        try:
            # Upload the image from memory
            self.storbinary(f"STOR {name}", img_buffer)
            print(f"Uploaded {name} successfully!")
    
        except Exception as e:
            print(f"Upload error to ftp: {e}")




