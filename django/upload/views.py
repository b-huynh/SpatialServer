from django.shortcuts import render
from django.http import HttpResponseRedirect
from django.utils import timezone

from .forms import UploadFileForm
from .files import handle_uploaded_file
from sfm.processing_manager import start_thread_processing_on_zip
from .models import ImageSet


def list(request):
    image_sets = ImageSet.objects.all()
    return render(request, 'list.html', {'image_sets': image_sets})


def index(request):
    if request.method == 'POST':
        form = UploadFileForm(request.POST, request.FILES)
        if form.is_valid():
            filename, ident = handle_uploaded_file(request.FILES['file'], form.cleaned_data['title'])
            image_set = ImageSet(desc='', ident=ident, status='uploaded', upload_date=timezone.now())
            image_set.save()
            start_thread_processing_on_zip(filename, image_set)
            return HttpResponseRedirect('/upload/list')
    else:
        form = UploadFileForm()

    return render(request, 'upload.html', {'form': form})