from django.shortcuts import render
from django.http import HttpResponse
from django.http import HttpResponseRedirect

from .forms import UploadFileForm
from .files import handle_uploaded_file
from sfm.processing_manager import start_thread_processing_on_zip


def success(request):
    return HttpResponse("Success!")


def index(request):
    if request.method == 'POST':
        form = UploadFileForm(request.POST, request.FILES)
        if form.is_valid():
            filename, ident = handle_uploaded_file(request.FILES['file'], form.cleaned_data['title'])
            start_thread_processing_on_zip(filename, ident)
            return HttpResponseRedirect('/upload/success')
    else:
        form = UploadFileForm()

    return render(request, 'upload.html', {'form': form})