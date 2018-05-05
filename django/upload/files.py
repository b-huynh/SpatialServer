

def handle_uploaded_file(f):
    with open('upload.zip', 'wb+') as destination:
        for chunk in f.chunks():
            destination.write(chunk)