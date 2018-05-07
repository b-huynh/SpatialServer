import time
import os

upload_dir = os.path.join('temp_files', 'zips/')


def handle_uploaded_file(f, title):
    timestr = time.strftime("%Y%m%d-%H%M%S")
    ident = title + '-' + timestr
    filename = os.path.join(upload_dir, ident + '.zip')

    with open(filename, 'wb+') as destination:
        for chunk in f.chunks():
            destination.write(chunk)
    return filename, ident
