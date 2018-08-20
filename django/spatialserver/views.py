from django.shortcuts import render


def register(request, ident1, ident2):
    return render(request, 'register.html', {'ident1': ident1, 'ident2': ident2})


def double(request, ident1, ident2):
    return render(request, 'potree_double.html', {'ident1': ident1, 'ident2': ident2})


def potree_reg(request, ident):
    return render(request, 'potree_reg.html', {'ident': ident})


def potree_reg_def(request):
    return render(request, 'potree_reg.html', {})
