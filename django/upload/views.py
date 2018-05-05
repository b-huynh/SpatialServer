from django.shortcuts import render
from django.http import HttpResponse
from django.http import HttpResponseRedirect

from .forms import UploadFileForm
from .files import handle_uploaded_file


def success(request):
    return HttpResponse("Success!")


def index(request):
    if request.method == 'POST':
        form = UploadFileForm(request.POST, request.FILES)
        if form.is_valid():
            handle_uploaded_file(request.FILES['file'])
            return HttpResponseRedirect('/upload/success')
    else:
        form = UploadFileForm()

    return render(request, 'upload.html', {'form': form})