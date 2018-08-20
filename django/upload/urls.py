from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path('list', views.image_set_list, name='image_set_list'),
    path('select', views.select, name='select'),
    path('align/new', views.new_align, name='new_align'),
    path('align/list', views.align_list, name='align_list')
]
