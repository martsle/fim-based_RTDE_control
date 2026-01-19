from ftplib import FTP

# FTP Server details
FTP_HOST = "10.152.213.30"
FTP_USER = "ftpuser"  # Use 'anonymous' if enabled
FTP_PASS = "FIM4AMisgreat"  # Leave empty for anonymous

def ftp_server_login():
    # Connect to FTP Server
    ftp = FTP(FTP_HOST)
    ftp.login(FTP_USER, FTP_PASS)
    return ftp

def create_dir_and_switch(ftp, study_code, exp_code, base_dir="Studies"):
    directories = [base_dir, study_code, exp_code]

    for dir in directories:
        # check if study directory exists, otherwise create_it
        if dir not in ftp.nlst():
            ftp.mkd(dir)
        ftp.cwd(dir)

    print(f"Changed to directory {'/'.join(directories)} successfully.")

def upload_img_from_buffer(ftp, img_buffer, name):
    # Uploads an image buffer directly to ftp
    try:
        # Upload the image from memory
        ftp.storbinary(f"STOR {name}", img_buffer)
        print(f"Uploaded {name} successfully!")
    
    except Exception as e:
        print(f"Upload error to ftp: {e}")




