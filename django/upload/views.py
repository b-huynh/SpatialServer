from django.shortcuts import render
from django.shortcuts import HttpResponse
from django.http import HttpResponseRedirect
from django.utils import timezone

import json

from .forms import UploadFileForm
from .files import handle_uploaded_file
from sfm.processing_manager import start_thread_processing_on_zip
from .models import ImageSet
from .models import Alignment
from .util import align


def image_set_list(request):
    image_sets = ImageSet.objects.all()
    return render(request, 'image_set_list.html', {'image_sets': image_sets})


def select(request):
    image_sets = ImageSet.objects.all()
    return render(request, 'select.html', {'image_sets': image_sets})


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


def new_align(request):
    if request.method == 'POST':
        points1 = json.loads(request.POST['points1_str'])
        points2 = json.loads(request.POST['points2_str'])
        trans_mat = align(points1, points2)
        mat_str = json.dumps(trans_mat)
        points1_str = json.dumps(points1)
        points2_str = json.dumps(points2)
        alignment = Alignment(ident1=request.POST['ident1'], ident2=request.POST['ident2'], matrix=mat_str,
                              points1=points1_str, points2=points2_str, upload_date=timezone.now())
        alignment.save()
        return HttpResponse('{"redirect_url": "/double/' + request.POST['ident1'] + '/' +
                            request.POST['ident2'] + '/?mat=' + mat_str + '"}')

    return HttpResponse('{}')


def align_list(request):
    aligns = Alignment.objects.all()
    return render(request, 'align_list.html', {'aligns': aligns})
