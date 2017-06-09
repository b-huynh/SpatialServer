var UCSB_COORDS = {
	'lat': 34.4139629,
	'lng': -119.8511357
}

function addMarkerAndWindow(map, val, name) {
	var latLng = new google.maps.LatLng(val.lat, val.lon);
	var content = [
	  name,
	  'LatLng: ' + latLng,
	].join('<br>');

    var infoWindow = new google.maps.InfoWindow();
    infoWindow.setContent(content);

    var marker = new google.maps.Marker({
    	position: latLng,
    	map: map,
    	title: name
    });
    marker.addListener('click', function() {
    	infoWindow.open(map, marker);
    })
}

function initMap() {

	var metadata;
	$.get('../metadata.json', function(data) {
		metadata = data;
		console.log(metadata);

		var ucsb = new google.maps.LatLng(UCSB_COORDS.lat, UCSB_COORDS.lng);
		map = new google.maps.Map(document.getElementById('map'), {
			center: ucsb,
			zoom: 17,
			clickableIcons: false
		});

		for (var datapoint in metadata) {
			addMarkerAndWindow(map, metadata[datapoint], datapoint);
		}
	}).fail(function() {
		alert("Could not retrieve metadata.");
	});
}
