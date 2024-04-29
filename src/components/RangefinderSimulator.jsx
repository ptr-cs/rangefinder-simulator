import React, { useRef, useEffect, useState } from 'react';
import Button from 'react-bootstrap/Button';
import Container from 'react-bootstrap/Container';
import Nav from 'react-bootstrap/Nav';
import Navbar from 'react-bootstrap/Navbar';
import Form from 'react-bootstrap/Form';
import NavDropdown from 'react-bootstrap/NavDropdown';
import Offcanvas from 'react-bootstrap/Offcanvas';
import './RangefinderSimulator.scss';
import { THREE, Threebox } from "threebox-plugin";
import "threebox-plugin/dist/threebox.css";
import "bootstrap/dist/css/bootstrap.css"
import mapboxgl from '!mapbox-gl'; // eslint-disable-line import/no-webpack-loader-syntax
import _, { range } from 'lodash'
import * as turf from '@turf/turf'
import MapboxGeocoder from '@mapbox/mapbox-gl-geocoder';
import '@mapbox/mapbox-gl-geocoder/dist/mapbox-gl-geocoder.css';
import Modal from 'react-bootstrap/Modal';

export default function RangefinderSimulator() {

  mapboxgl.accessToken = process.env.REACT_APP_MAPBOX_TOKEN;

  const mapContainerRef = useRef(null);
  const mapContainerRangeRef = useRef(null);
  const [map, setMap] = useState(null);
  const [rangeMap, setRangeMap] = useState(null);
  const [threebox, setThreebox] = useState(null);
  const [rangefinder, setRangefinder] = useState(null);
  const [laser, setLaser] = useState(null);
  const [lineSourceData, setLineSourceData] = useState({
    type: 'Feature',
    geometry: {
      type: 'LineString',
      properties: {},
      coordinates: [[-122.450648, 37.701857], [-122.439287109375, 37.701857]]
    }
  });

  const MAP_INITIAL_CENTER = { lat: 37.701857, lng: -122.450648 }
  const MAP_INITIAL_ZOOM = 15
  const MAP_INITIAL_PITCH = 0
  const MAP_INITIAL_BEARING = 0
  const MAP_INITIAL_ALTITUDE = 0

  const [mapCenter, setMapCenter] = useState(MAP_INITIAL_CENTER);
  const [mapZoom, setMapZoom] = useState(MAP_INITIAL_ZOOM);
  const [mapPitch, setMapPitch] = useState(MAP_INITIAL_PITCH);
  const [mapBearing, setMapBearing] = useState(MAP_INITIAL_BEARING);
  const [mapAltitude, setMapAltitude] = useState(MAP_INITIAL_ALTITUDE);
  
  const [showMapSettings, setShowMapSettings] = useState(true);
  const [showRangeSettings, setShowRangeSettings] = useState(true);
  const [showMap, setShowMap] = useState(true);
  const [showRanged, setShowRanged] = useState(true);

  const [mapRangeCenter, setMapRangeCenter] = useState(MAP_INITIAL_CENTER);

  const [prevMapCenter, setPrevMapCenter] = useState({ lat: 0.0, lng: 0.0 });
  const [prevMapZoom, setPrevMapZoom] = useState(0);
  const [prevMapPitch, setPrevMapPitch] = useState(0);
  const [prevMapBearing, setPrevMapBearing] = useState(0);

  const RANGEFINDER_INITIAL_COORDS = [-122.450648, 37.701857, 500]
  const RANGEFINDER_INITIAL_ROTATION = { x: 0, y: 0, z: 0 }
  const RANGEFINDER_INITIAL_SCALE = 0.03
  const RANGEFINDER_INITIAL_MAX_RANGE = 1000.0

  const [rangefinderCoords, setRangefinderCoords] = useState(RANGEFINDER_INITIAL_COORDS);
  const [rangefinderRotation, setRangefinderRotation] = useState(RANGEFINDER_INITIAL_ROTATION);
  const [rangefinderScale, setRangefinderScale] = useState(RANGEFINDER_INITIAL_SCALE);
  const [rangefinderMaxRange, setRangefinderMaxRange] = useState(RANGEFINDER_INITIAL_MAX_RANGE);
  const [rangefinderRange, setRangefinderRange] = useState(0);

  const rangefinderRotationDebounced = useRef(_.debounce(setRangefinderRotation, 1));

  const [debugCounter, setDebugCounter] = useState(0);

  const setInterpolatedIntersectionDebounced = useRef(_.debounce(getInterpolatedIntersection, 100));

  const setMapRangeCenterDebounced = useRef(_.debounce(setMapRangeCenterAndUpdateMap, 200));

  const setMapCenterDebounced = useRef(_.debounce(setMapCenter, 250));
  const setMapCenterInputDebounced = useRef(_.debounce(setMapCenter, 10));
  const setMapZoomDebounced = useRef(_.debounce(setMapZoom, 250));
  const setMapZoomInputDebounced = useRef(_.debounce(setMapZoom, 10));
  const setMapPitchDebounced = useRef(_.debounce(setMapPitch, 250));
  const setMapPitchInputDebounced = useRef(_.debounce(setMapPitch, 10));
  const setMapBearingDebounced = useRef(_.debounce(setMapBearing, 250));
  const setMapBearingInputDebounced = useRef(_.debounce(setMapBearing, 10));
  
  const [openDropdown, setOpenDropdown] = useState(false);
  const closingTimerRef = useRef(null);
  
  const [showHelpModal, setShowHelpModal] = useState(false);

  const handleCloseHelpModal = () => setShowHelpModal(false);
  const handleShowHelpModal = () => showHelpModal ? setShowHelpModal(false) : setShowHelpModal(true);
  
  const handleSelect = (event) => {
    event.preventDefault(); // Prevent default
    // Logic for item click
    setOpenDropdown(true); // Keep open
  };
  
  const handleMouseEnter = () => {
    if (closingTimerRef.current) {
      clearTimeout(closingTimerRef.current); // Cancel any pending close if re-entering
    }
    setOpenDropdown(true);
  };

  const handleMouseLeave = () => {
    // Start a timer to close the dropdown, gives time to move to the dropdown content
    closingTimerRef.current = setTimeout(() => {
      setOpenDropdown(false);
    }, 500); // 500ms delay before closing, adjust as needed
  };

  // Function to update line position
  const updateLinePosition = (newLinePosition) => {
    setLineSourceData({
      type: 'Feature',
      properties: {},
      geometry: {
        type: 'LineString',
        coordinates: newLinePosition
      }
    });
  };

  const updateLinePositionDebounced = useRef(_.debounce(updateLinePosition, 100));

  const onDraggedObject = (e) => {
    setRangefinderCoords(e.detail.draggedObject.coordinates)
    var newRotation = { x: (e.detail.draggedObject.rotation._x * (180 / Math.PI)).toFixed(6), y: (e.detail.draggedObject.rotation._y * (180 / Math.PI)).toFixed(6), z: (e.detail.draggedObject.rotation._z * (180 / Math.PI)).toFixed(6) }
    setRangefinderRotation(newRotation)
  }

  function createLaser(tb) {
    try {
      if (tb)
        tb.remove(laser);
    } catch (e) {
      console.log(e)
    }
    // Define the start and end coordinates for the line
    const startCoord = rangefinderCoords
    const destination = calculateDestination(startCoord[1], startCoord[0], 0, rangefinderMaxRange);
    const endCoord = [destination.lon, destination.lat, rangefinderCoords[2]];

    // Convert coordinates to Threebox world positions
    const startPos = tb.projectToWorld(startCoord);
    const endPos = tb.projectToWorld(endCoord);

    const points = [];
    points.push(new THREE.Vector3(startPos.x, startPos.y, startPos.z));
    points.push(new THREE.Vector3(endPos.x, endPos.y, endPos.z));

    // Create a geometry for the line
    const geometry = new THREE.BufferGeometry().setFromPoints(points);

    // Create a material for the line
    const material = new THREE.LineBasicMaterial({ color: 0xff0000, transparent: true, opacity: .5 });

    // Create the line using the geometry and material
    const line = new THREE.Line(geometry, material);

    setLaser(line)

    // Add the line to the Threebox scene
    tb.add(line);
  }

  useEffect(() => {
    const initMap = new mapboxgl.Map({
      container: mapContainerRef.current,
      style: 'mapbox://styles/mapbox/streets-v11',
      center: [mapCenter.lng, mapCenter.lat],
      zoom: mapZoom,
      antialias: true // create the gl context with MSAA antialiasing, so custom layers are antialiased
    });

    const initRangeMap = new mapboxgl.Map({
      container: mapContainerRangeRef.current,
      style: 'mapbox://styles/mapbox/streets-v11',
      center: [mapCenter.lng, mapCenter.lat],
      zoom: mapZoom,
      antialias: true // create the gl context with MSAA antialiasing, so custom layers are antialiased
    });

    // Add the control to the map.
    initMap.addControl(
      new MapboxGeocoder({
        accessToken: mapboxgl.accessToken,
        mapboxgl: mapboxgl
      })
    );
    initMap.addControl(new mapboxgl.NavigationControl());
    initMap.addControl(new mapboxgl.ScaleControl());

    initRangeMap.addControl(new mapboxgl.NavigationControl());
    initRangeMap.addControl(new mapboxgl.ScaleControl());



    // eslint-disable-next-line no-undef
    const tb = (window.tb = new Threebox(
      initMap,
      initMap.getCanvas().getContext('webgl'),
      {
        defaultLights: true, enableSelectingObjects: true, enableDraggingObjects: true, enableRotatingObjects: true, enableTooltips: true
      }
    ));

    tb.altitudeStep = 1.0;

    setThreebox(tb)

    createLaser(tb)

    initMap.on('load', () => {
      var camera = initMap.getFreeCameraOptions();
      setMapAltitude(camera._position.toAltitude().toFixed(6))
    })

    initMap.on('style.load', () => {
      // Insert the layer beneath any symbol layer.
      const layers = initMap.getStyle().layers;
      const labelLayerId = layers.find(
        (layer) => layer.type === 'symbol' && layer.layout['text-field']
      ).id;

      // The 'building' layer in the Mapbox Streets
      // vector tileset contains building height data
      // from OpenStreetMap.
      initMap.addLayer(
        {
          'id': 'add-3d-buildings',
          'source': 'composite',
          'source-layer': 'building',
          'filter': ['==', 'extrude', 'true'],
          'type': 'fill-extrusion',
          'minzoom': 15,
          'paint': {
            'fill-extrusion-color': '#aaa',

            // Use an 'interpolate' expression to
            // add a smooth transition effect to
            // the buildings as the user zooms in.
            'fill-extrusion-height': [
              'interpolate',
              ['linear'],
              ['zoom'],
              15,
              0,
              15.05,
              ['get', 'height']
            ],
            'fill-extrusion-base': [
              'interpolate',
              ['linear'],
              ['zoom'],
              15,
              0,
              15.05,
              ['get', 'min_height']
            ],
            'fill-extrusion-opacity': 0.6
          }
        },
        labelLayerId
      );


      initMap.addSource('mapbox-dem', {
        'type': 'raster-dem',
        'url': 'mapbox://mapbox.mapbox-terrain-dem-v1',
        'tileSize': 512,
        'maxzoom': 14
      });
      // add the DEM source as a terrain layer with exaggerated height
      initMap.setTerrain({ 'source': 'mapbox-dem', 'exaggeration': 1.5 }); // 'exaggeration': 1.5

      initMap.addSource('lineSource', {
        'type': 'geojson',
        'data': lineSourceData
      });

      // // Add a layer to visualize the line
      // initMap.addLayer({
      //   'id': 'lineLayer',
      //   'type': 'line',
      //   'source': 'lineSource',
      //   'layout': {},
      //   'paint': {
      //     'line-width': 8,
      //     'line-color': '#007cbf' // Line color
      //   }
      // });

      initMap.addLayer({
        id: 'custom-threebox-model',
        type: 'custom',
        renderingMode: '3d',
        onAdd: function () {
          // Creative Commons License attribution: https://kenney.nl/
          const options = {
            obj: 'rangefinder.gltf',
            type: 'gltf',
            anchor: 'center',
            adjustment: { x: .5, y: 0, z: -0.5 },
            scale: { x: 500, y: 500, z: 500 },
            units: 'meters',
            rotation: { x: 90, y: -90, z: 0 }
          };

          tb.loadObj(options, (model) => {
            model.setCoords(rangefinderCoords);
            model.setRotation(rangefinderRotation);
            model.addTooltip("Rangefinder", true);
            tb.add(model);
            model.addEventListener('ObjectDragged', onDraggedObject, false);
            setRangefinder(model)
          });
        },

        render: function () {
          tb.update();
        }
      });
    });

    initMap.on('move', () => {
      setMapCenterDebounced.current({
        lat: initMap.getCenter().lat.toFixed(6),
        lng: initMap.getCenter().lng.toFixed(6),
      });
      setMapBearingDebounced.current(initMap.getBearing().toFixed(6))
      setMapPitchDebounced.current(initMap.getPitch().toFixed(6))
      var camera = initMap.getFreeCameraOptions();
      setMapAltitude(camera._position.toAltitude().toFixed(6))
    });

    initMap.on('zoom', (e) => {
      setMapZoomDebounced.current(initMap.getZoom().toFixed(6));
    })

    setMap(initMap);
    setRangeMap(initRangeMap)

    return () => {
      if (closingTimerRef.current) {
        clearTimeout(closingTimerRef.current);
      }
      
      try {
        initMap.remove();
        initRangeMap.remove();
      } catch { }
    }
  }, []);

  function calculateDestination(lat, lon, bearing, distance) {
    const R = 6371e3; // Earth's radius in meters
    const bearingRad = (bearing + 90) * Math.PI / 180; // Convert bearing to radians

    const latRad = lat * Math.PI / 180; // Convert latitude to radians
    const lonRad = lon * Math.PI / 180; // Convert longitude to radians

    // Calculate the destination coordinates
    const latEndRad = Math.asin(Math.sin(latRad) * Math.cos(distance / R) +
      Math.cos(latRad) * Math.sin(distance / R) * Math.cos(bearingRad));
    const lonEndRad = lonRad + Math.atan2(Math.sin(bearingRad) * Math.sin(distance / R) * Math.cos(latRad),
      Math.cos(distance / R) - Math.sin(latRad) * Math.sin(latEndRad));

    // Convert back to degrees
    const latEnd = latEndRad * 180 / Math.PI;
    const lonEnd = lonEndRad * 180 / Math.PI;

    return { lat: latEnd, lon: lonEnd };
  }

  // Function to sample elevation along a 3D line and identify intersection with terrain
  function findTerrainIntersection(lat, lng, alt, map, threebox) {
    // console.log("line lat/lng/alt")
    // console.log(lat)
    // console.log(lng)
    // console.log(alt)
    //const vertices = lineGeometry.getAttribute('position').array;
    let intersectionPoint = null;

    // Sample terrain elevation at the geographic coordinates
    const terrainElevation = map.queryTerrainElevation([lng, lat], { exaggerated: true });

    // Determine if the line segment intersects the terrain
    if (alt <= terrainElevation) {
      // The line has intersected the terrain. Calculate the exact intersection point if necessary.
      // For simplicity, this example marks the first intersection point and breaks.
      intersectionPoint = { lat: lat, lng: lng, alt: terrainElevation }
    }

    return intersectionPoint;
  }

  const handleMapCenterInputChange = (e) => {
    const { name, value } = e.target;
    if (isNaN(value))
      return
    setMapCenterInputDebounced.current((prevCenter) => ({
      ...prevCenter,
      [name]: parseFloat(value).toFixed(6),
    }));
  };

  const handleMapZoomInputChange = (e) => {
    if (isNaN(e.target.value))
      return
    setMapZoomInputDebounced.current(parseFloat(e.target.value).toFixed(6));
  };

  const handleMapPitchInputChange = (e) => {
    if (isNaN(e.target.value))
      return
    setMapPitchInputDebounced.current(parseFloat(e.target.value).toFixed(6));
  };

  const handleMapBearingInputChange = (e) => {
    if (isNaN(e.target.value))
      return
    setMapBearingInputDebounced.current(parseFloat(e.target.value).toFixed(6));
  };

  const handleRangefinderCoordsInputChangeLat = (e) => {
    if (isNaN(e.target.value))
      return
    setRangefinderCoords([rangefinderCoords[0], Number(parseFloat(e.target.value).toFixed(6)), rangefinderCoords[2]]);
  };

  const handleRangefinderCoordsInputChangeLng = (e) => {
    if (isNaN(e.target.value))
      return
    setRangefinderCoords([Number(parseFloat(e.target.value).toFixed(6)), rangefinderCoords[1], rangefinderCoords[2]]);
  };

  const handleRangefinderCoordsInputChangeAlt = (e) => {
    if (isNaN(e.target.value))
      return
    setRangefinderCoords([rangefinderCoords[0], rangefinderCoords[1], Number(parseFloat(e.target.value).toFixed(6))]);
  };

  const handleRangefinderRotationInputChangeX = (e) => {
    if (isNaN(e.target.value))
      return
    setRangefinderRotation({ x: parseFloat(e.target.value).toFixed(6), y: rangefinderRotation.y, z: rangefinderRotation.z });
  };

  const handleRangefinderRotationInputChangeY = (e) => {
    if (isNaN(e.target.value))
      return
    setRangefinderRotation({ x: rangefinderRotation.x, y: parseFloat(e.target.value).toFixed(6), z: rangefinderRotation.z });
  };

  const handleRangefinderRotationInputChangeZ = (e) => {
    if (isNaN(e.target.value))
      return
    setRangefinderRotation({ x: rangefinderRotation.x, y: rangefinderRotation.y, z: parseFloat(e.target.value).toFixed(6) });
  };

  const handleRangefinderRangeInputChange = (e) => {
    if (isNaN(e.target.value))
      return
    setRangefinderMaxRange(parseFloat(e.target.value).toFixed(6));
  }

  // Handler to update scale from input
  const handleRangefinderScaleInputChange = (e) => {
    const newScale = parseFloat(e.target.value).toFixed(3);
    setRangefinderScale(newScale);
    updateModelScale(newScale); // Apply scale immediately
  };

  const handleRangefinderReset = (e) => {
    setRangefinderCoords(RANGEFINDER_INITIAL_COORDS)
    setRangefinderRotation({ x: 0.00000000001, y: 0.00000000001, z: 0.00000000001 })
    rangefinderRotationDebounced.current(RANGEFINDER_INITIAL_ROTATION)
    setRangefinderScale(RANGEFINDER_INITIAL_SCALE)
    setRangefinderMaxRange(RANGEFINDER_INITIAL_MAX_RANGE)
  }

  const handleRangefinderDebugReset = (e) => {
    createLaser(threebox)
    // handleRangefinderReset()
  }

  function getLaserEndCoordinates() {
    // Get vertices from buffer geometry
    const verticesFromGeometry = getVerticesFromBufferGeometry(laser.geometry);
    return threebox.unprojectFromWorld(verticesFromGeometry[1]);
  }

  function getLaserEndCoordinatesParams(laser, threebox) {
    // Get vertices from buffer geometry
    const verticesFromGeometry = getVerticesFromBufferGeometry(laser.geometry);
    return threebox.unprojectFromWorld(verticesFromGeometry[1]);
  }

  const handleRangefinderMarkMap = (e) => {

    if (rangefinderRange === 0)
      return

    const laserEnd = getLaserEndCoordinates()

    // Create a popup for the tooltip
    const popup = new mapboxgl.Popup({ offset: 25 }).setText(
      "Lat: " + parseFloat(laserEnd[1]).toFixed(6) + " º " +
      "Lng: " + parseFloat(laserEnd[1]).toFixed(6) + " º " +
      "Alt: " + parseFloat(laserEnd[2]).toFixed(6) + " m " +
      "Range: " + rangefinderRange + " m " // Replace with your tooltip text
    );

    const marker2 = new mapboxgl.Marker({ color: 'red' })
      .setLngLat([laserEnd[0], laserEnd[1]])
      .setPopup(popup)
      .addTo(map);
  }

  const handleMapReset = (e) => {
    setMapCenter(MAP_INITIAL_CENTER)
    setMapZoom(MAP_INITIAL_ZOOM)
    setMapPitch(MAP_INITIAL_PITCH)
    setMapBearing(MAP_INITIAL_BEARING)
  }

  const handleRangefinderJumpToMap = (e) => {
    var currentAlt = rangefinderCoords[2]
    var sampledAlt = map.queryTerrainElevation([mapCenter.lng, mapCenter.lat], { exaggerated: true })
    var newAlt = currentAlt > sampledAlt ? currentAlt : sampledAlt + 500
    setRangefinderCoords([mapCenter.lng, mapCenter.lat, newAlt]);
  }

  const handleJumpToRangefinder = (e) => {
    setMapCenterInputDebounced.current({ lat: rangefinderCoords[1], lng: rangefinderCoords[0] })
    setMapAltitude(rangefinderCoords[2] * 2)
  }

  // Function to update the model's scale
  const updateModelScale = (newScale) => {
    if (rangefinder && threebox) {
      rangefinder.scale.set(newScale, newScale, newScale);
      threebox.update();
    }
  };

  // Function to get vertices from a THREE.BufferGeometry() object
  function getVerticesFromBufferGeometry(bufferGeometry) {
    const vertices = [];

    // Get position attribute from buffer geometry
    const positions = bufferGeometry.attributes.position.array;

    // Iterate over position attribute to extract vertices
    for (let i = 0; i < positions.length; i += 3) {
      const x = positions[i];
      const y = positions[i + 1];
      const z = positions[i + 2];
      vertices.push(new THREE.Vector3(x, y, z));
    }

    return vertices;
  }

  function calculate3DDistance(coord1, coord2) {
    // Extract longitude, latitude, and altitude for each coordinate
    const [lon1, lat1, alt1] = coord1;
    const [lon2, lat2, alt2] = coord2;

    // Calculate the 2D distance between the points on the surface
    const from = turf.point([lon1, lat1]);
    const to = turf.point([lon2, lat2]);
    const distance2D = turf.distance(from, to, 'meters');

    // Calculate the difference in altitude
    const altitudeDifference = Math.abs(alt1 - alt2);

    // Use the Pythagorean theorem to calculate the 3D distance
    const distance3D = Math.sqrt((distance2D * distance2D) + (altitudeDifference * altitudeDifference));

    return distance3D;
  }

  /**
 * Calculate a new point that lies at a specified distance along the length of a 3D line.
 * 
 * @param {Array} startCoord - The start coordinate of the line [longitude, latitude, altitude].
 * @param {Array} endCoord - The end coordinate of the line [longitude, latitude, altitude].
 * @param {Number} distance - The distance from the startCoor of the new coordinate
 * @returns {Array} The new coordinate [longitude, latitude, altitude].
 */
  function calculateCoordinateFromDistance(startCoord, endCoord, distance) {
    // Destructure the start and end coordinates
    const [startLng, startLat, startAlt] = startCoord;
    const [endLng, endLat, endAlt] = endCoord;

    // Calculate the differences in longitude, latitude, and altitude
    const lngDiff = endLng - startLng;
    const latDiff = endLat - startLat;
    const altDiff = endAlt - startAlt;

    // Calculate the new coordinate by adding the distance of the differences to the start coordinate
    const newLng = startLng + (lngDiff * distance);
    const newLat = startLat + (latDiff * distance);
    const newAlt = startAlt + (altDiff * distance);

    return [newLng, newLat, newAlt];
  }

  function setMapRangeCenterAndUpdateMap(laser, threebox, rangeMap) {
    console.log("here")
    const newLaserEnd = getLaserEndCoordinatesParams(laser, threebox)
    setMapRangeCenter({ lat: newLaserEnd[1], lng: newLaserEnd[0] })
    rangeMap.jumpTo({
      center: [newLaserEnd[0], newLaserEnd[1]]
    })
  }

  function getInterpolatedIntersection(laser, laserStartVector3, laserEndVector3, laserStart, laserEnd, rangefinderMaxRange, map, threebox) {
    var segmentStart = laserStart
    var segmentEnd = laserEnd
    var newSegmentEnd = null
    for (var i = 0; i < rangefinderMaxRange; i++) {
      // console.log("running loop " + i)
      newSegmentEnd = calculateCoordinateFromDistance(segmentStart, segmentEnd, i / rangefinderMaxRange)
      const intersectionResult = findTerrainIntersection(newSegmentEnd[1], newSegmentEnd[0], newSegmentEnd[2], map, threebox)
      // console.log("intersectionResult")
      // console.log(intersectionResult)
      if (intersectionResult) {

        const distance3d = calculate3DDistance([intersectionResult.lng, intersectionResult.lat, intersectionResult.alt], laserStart)
        // console.log("distance3d")
        // console.log(distance3d)
        if (distance3d <= rangefinderMaxRange) {
          setRangefinderRange(parseFloat(distance3d).toFixed(6))

          const newLaserEnd = threebox.projectToWorld([intersectionResult.lng, intersectionResult.lat, intersectionResult.alt]);
          // console.log("newLaserEnd")
          // console.log(newLaserEnd)
          const newLaserEndVector3 = new THREE.Vector3(newLaserEnd.x, newLaserEnd.y, newLaserEnd.z)
          const updatedVertices = [laserStartVector3, newLaserEndVector3]
          laser.geometry.setFromPoints(updatedVertices);
          // console.log("updatedVertices")
          // console.log(updatedVertices)
          laser.geometry.attributes.position.needsUpdate = true; // required after the first render
          break;
        }
      } else {
        setRangefinderRange(0)
        const updatedVertices = [laserStartVector3, laserEndVector3]
        laser.geometry.setFromPoints(updatedVertices);
        laser.geometry.attributes.position.needsUpdate = true;
      }
    }
  }

  useEffect(() => {
    if (rangefinder && laser && rangefinderCoords && rangefinderRotation && rangefinderScale && rangefinderMaxRange) {
      rangefinder.setCoords(rangefinderCoords)
      rangefinder.setRotation(rangefinderRotation)
      updateModelScale(rangefinderScale);
      // Define the start and end coordinates for the laser
      const startCoord = rangefinderCoords
      const destination = calculateDestination(startCoord[1], startCoord[0], 0, rangefinderMaxRange);
      const endCoord = [destination.lon, destination.lat, rangefinderCoords[2]];
      // Convert coordinates to Threebox world positions
      const startPos = threebox.projectToWorld(startCoord);
      const endPos = threebox.projectToWorld(endCoord);
      // const startPosMap = map.project(startCoord);
      // const endPosMap = map.project(endCoord);
      // const points = [];

      var startPointVector = new THREE.Vector3(startPos.x, startPos.y, startPos.z)
      var endPointVector = new THREE.Vector3(endPos.x, endPos.y, endPos.z)

      // Define the rotation angles in radians
      const angleX = THREE.MathUtils.degToRad(rangefinderRotation.x);
      const angleY = THREE.MathUtils.degToRad(rangefinderRotation.y);
      const angleZ = THREE.MathUtils.degToRad(rangefinderRotation.z);

      // Create a rotation matrix
      const rotationMatrix = new THREE.Matrix4();
      rotationMatrix.makeRotationFromEuler(new THREE.Euler(angleX, angleY, angleZ));

      // Define the line's vertices
      const vertices = [
        startPointVector,
        endPointVector
      ];

      // Translate the line to the origin
      const origin = vertices[0].clone();
      const translationMatrix = new THREE.Matrix4().makeTranslation(-origin.x, -origin.y, -origin.z);

      // Apply rotation
      const rotatedVertices = vertices.map(vertex => {
        const vertexVec = new THREE.Vector3().copy(vertex);
        vertexVec.applyMatrix4(translationMatrix); // Translate to origin
        vertexVec.applyMatrix4(rotationMatrix);   // Apply rotation
        return vertexVec;
      });

      // Translate the line back to its original position
      const inverseTranslationMatrix = new THREE.Matrix4().makeTranslation(origin.x, origin.y, origin.z);
      rotatedVertices.forEach(vertex => vertex.applyMatrix4(inverseTranslationMatrix));

      laser.geometry.setFromPoints(rotatedVertices); // test comment

      // Get vertices from buffer geometry
      const verticesFromGeometry = getVerticesFromBufferGeometry(laser.geometry);

      const laserStart = threebox.unprojectFromWorld(verticesFromGeometry[0]);
      const laserEnd = threebox.unprojectFromWorld(verticesFromGeometry[1]);
      updateLinePositionDebounced.current([[laserStart[0], laserStart[1]], [laserEnd[0], laserEnd[1]]])

      setInterpolatedIntersectionDebounced.current(laser, verticesFromGeometry[0], verticesFromGeometry[1], laserStart, laserEnd, rangefinderMaxRange, map, threebox)

      setMapRangeCenterDebounced.current(laser, threebox, rangeMap)

    }
  }, [rangefinderCoords, rangefinderRotation, rangefinderScale, rangefinderMaxRange]);

  useEffect(() => {
    if (map && lineSourceData) {
      try {
        map.getSource('lineSource').setData(lineSourceData);
      } catch { }

    }
  }, [lineSourceData, map]);

  useEffect(() => {
    if (map && !isNaN(mapCenter.lat) && !isNaN(mapCenter.lng) && !isNaN(mapZoom)
      && ((prevMapZoom !== parseFloat(mapZoom).toFixed(6) ||
        prevMapCenter.lat !== parseFloat(mapCenter.lat).toFixed(6) ||
        prevMapCenter.lng !== parseFloat(mapCenter.lng).toFixed(6)) ||
        prevMapPitch !== parseFloat(mapPitch).toFixed(6) ||
        prevMapBearing !== parseFloat(mapBearing).toFixed(6))) {
      if (map !== null) {
        map.jumpTo({
          center: [parseFloat(mapCenter.lng).toFixed(6), parseFloat(mapCenter.lat).toFixed(6)],
          zoom: parseFloat(mapZoom).toFixed(6),
          pitch: parseFloat(mapPitch).toFixed(6),
          bearing: parseFloat(mapBearing).toFixed(6),
          essential: true,
          duration: 0
        });
      }
    }
    setPrevMapZoom(parseFloat(mapZoom).toFixed(6))
    setPrevMapCenter({
      lat: parseFloat(mapCenter.lat).toFixed(6),
      lng: parseFloat(mapCenter.lng).toFixed(6),
    })
    setPrevMapPitch(parseFloat(mapPitch).toFixed(6))
    setPrevMapBearing(parseFloat(mapBearing).toFixed(6))
    setDebugCounter(debugCounter + 1)

  }, [mapCenter, mapZoom, mapPitch, mapBearing]);

  function handleOnClick(e) {
    console.log("handleOnClick")
    e.preventDefault()
    return;
  }
  
  function handleShowMapSettings(e) {
    showMapSettings ? setShowMapSettings(false) : setShowMapSettings(true)
  }
  
  function handleShowRangeSettings(e) {
    showRangeSettings ? setShowRangeSettings(false) : setShowRangeSettings(true)
  }
  
  function handleShowMap(e) {
    showMap ? setShowMap(false) : setShowMap(true)
  }
  
  function handleShowRanged(e) {
    showRanged ? setShowRanged(false) : setShowRanged(true)
  }

  return (
    <div className='top-level-container'>
{['md'].map((expand) => (
        <Navbar key={expand} expand={expand} className="bg-body-tertiary mb-3" bg="primary" data-bs-theme="dark">
          <Container fluid>
            <Navbar.Brand href="#">Rangefinder Simulator</Navbar.Brand>
            <Navbar.Toggle aria-controls={`offcanvasNavbar-expand-${expand}`} />
            <Navbar.Offcanvas
              id={`offcanvasNavbar-expand-${expand}`}
              aria-labelledby={`offcanvasNavbarLabel-expand-${expand}`}
              placement="end"
            >
              <Offcanvas.Header closeButton>
                <Offcanvas.Title id={`offcanvasNavbarLabel-expand-${expand}`}>
                  Rangefinder Simulator
                </Offcanvas.Title>
              </Offcanvas.Header>
              <Offcanvas.Body>
                <Nav className="justify-content-end flex-grow-1 pe-3">
                <NavDropdown
                    title="Settings"
                    id={`offcanvasNavbarDropdown-expand-${expand}`}
                    show={openDropdown}
                    onMouseEnter={handleMouseEnter}
                    onMouseLeave={handleMouseLeave}
                  >
                    <NavDropdown.Item href="#action3">
                      <Form>
                        <Form.Check // prettier-ignore
                          type={'switch'}
                          id={`default-checkbox-1`}
                          onChange={handleShowMapSettings}
                          checked={showMapSettings}
                          label={`Show Map Settings`}
                        />
                      </Form>
                    </NavDropdown.Item>
                    <NavDropdown.Item href="#action3">
                      <Form>
                      <Form.Check // prettier-ignore
                        type={'switch'}
                        id={`default-checkbox-2`}
                        label={`Show Rangefinder Settings`}
                        onChange={handleShowRangeSettings}
                        checked={showRangeSettings}
                      />
                      </Form>
                    </NavDropdown.Item>
                    <NavDropdown.Item href="#action3">
                      <Form>
                      <Form.Check // prettier-ignore
                        type={'switch'}
                        id={`default-checkbox-3`}
                        label={`Show Map View`}
                        onChange={handleShowMap}
                        checked={showMap}
                      />
                      </Form>
                    </NavDropdown.Item>
                    <NavDropdown.Item href="#action3">
                      <Form>
                      <Form.Check // prettier-ignore
                        type={'switch'}
                        id={`default-checkbox-4`}
                        label={`Show Ranged View`}
                        onChange={handleShowRanged}
                        checked={showRanged}
                      />
                      </Form>
                    </NavDropdown.Item>
                    <NavDropdown.Divider />
                    <NavDropdown.Item onClick={handleMapReset} className='settings-with-warning'>
                      <div className='reset-button-label-settings settings-warning'></div>
                        Reset Map
                    </NavDropdown.Item>
                    <NavDropdown.Item onClick={handleRangefinderReset} className='settings-with-warning'>
                    <div className='reset-button-label-settings settings-warning'></div>
                      Reset Rangefinder
                    </NavDropdown.Item>
                    <NavDropdown.Item onClick={handleRangefinderDebugReset} className='settings-with-warning'>
                    <div className='reset-button-label-settings settings-warning'></div>
                      Reset Laser
                    </NavDropdown.Item>
                  </NavDropdown>
                  <Nav.Link onClick={handleShowHelpModal}>Help</Nav.Link>
                  <Nav.Link href="https://github.com/ptr-cs/rangefinder-simulator" target="_blank" rel="noopener noreferrer">Source</Nav.Link>
                </Nav>
              </Offcanvas.Body>
            </Navbar.Offcanvas>
          </Container>
        </Navbar>
      ))}
      <div className='parent-div' onClick={() => setOpenDropdown(false)}>

      { showMapSettings && <div className='sim-controls'>
            <h3>Map</h3>
            <div className='label-control-pair'>
              <Form.Label>Latitude</Form.Label>
              <Form.Control min='-90' max='90' step="0.000001" name='lat' type='number' value={mapCenter.lat} onChange={handleMapCenterInputChange} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair'>
              <Form.Label>Longitude</Form.Label>
              <Form.Control min='-180' max='180' step="0.000001" name='lng' type='number' value={mapCenter.lng} onChange={handleMapCenterInputChange} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair altitude-pair'>
              <Form.Label>Altitude</Form.Label>
              <Form.Control value={mapAltitude} readOnly={true} />
              <Form.Label className='meters-symbol-superscript'>m</Form.Label>
            </div>
            <div className='label-control-pair empty-label-margin-right'>
              <Form.Label>Zoom</Form.Label>
              <Form.Control step="0.1" type='number' value={mapZoom} onChange={handleMapZoomInputChange} />
              <Form.Label></Form.Label>
            </div>
            <div className='label-control-pair'>
              <Form.Label>Pitch</Form.Label>
              <Form.Control min='-180' max='180' step="0.1" name='pitch' type='number' value={mapPitch} onChange={handleMapPitchInputChange} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair'>
              <Form.Label>Bearing</Form.Label>
              <Form.Control step="0.1" type='number' value={mapBearing} name='bearing' onChange={handleMapBearingInputChange} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair reset-button-pair empty-label-margin-right'>
              <Form.Label className='action-button-label'></Form.Label>
              <Button variant="primary" title='Moves the map to the rangefinder location' onClick={handleJumpToRangefinder}>Jump to Rangef.</Button>
              <Form.Label></Form.Label>
            </div>
            <div className='label-control-pair reset-button-pair empty-label-margin-right'>
              <Form.Label className='reset-button-label action-button-label'></Form.Label>
              <Button variant="danger" title='Resets the map parameters' onClick={handleMapReset}>Reset</Button>
              <Form.Label></Form.Label>
            </div>
          </div> }
          { showRangeSettings && <div className="sim-controls ">
            <h3>Rangefinder</h3>
            <div className='label-control-pair'>
              <Form.Label>Latitude</Form.Label>
              <Form.Control type='number' min='-90' max='90' step="0.000001" value={rangefinderCoords[1]} onChange={handleRangefinderCoordsInputChangeLat} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair'>
              <Form.Label>Longitude</Form.Label>
              <Form.Control type='number' min='-180' max='180' step="0.000001" value={rangefinderCoords[0]} onChange={handleRangefinderCoordsInputChangeLng} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair altitude-pair'>
              <Form.Label>Altitude</Form.Label>
              <Form.Control type='number' min='0' max='10000' value={rangefinderCoords[2]} onChange={handleRangefinderCoordsInputChangeAlt} />
              <Form.Label className='meters-symbol-superscript'>m</Form.Label>
            </div>
            <div className='label-control-pair'>
              <Form.Label>Rotation X</Form.Label>
              <Form.Control step="1" type='number' value={rangefinderRotation.x} onChange={handleRangefinderRotationInputChangeX} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair'>
              <Form.Label>Rotation Y</Form.Label>
              <Form.Control step="1" type='number' value={rangefinderRotation.y} onChange={handleRangefinderRotationInputChangeY} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair'>
              <Form.Label>Rotation Z</Form.Label>
              <Form.Control step="1" type='number' value={rangefinderRotation.z} onChange={handleRangefinderRotationInputChangeZ} />
              <Form.Label>º</Form.Label>
            </div>
            <div className='label-control-pair altitude-pair'>
              <Form.Label>MaxRange</Form.Label>
              <Form.Control step="1.0" type='number' value={rangefinderMaxRange} onChange={handleRangefinderRangeInputChange} />
              <Form.Label className='meters-symbol-superscript'>m</Form.Label>
            </div>
            <div className='label-control-pair altitude-pair'>
              <Form.Label>Range</Form.Label>
              <Form.Control step="0.000001" type='number' value={rangefinderRange} readOnly={true} />
              <Form.Label className='meters-symbol-superscript'>m</Form.Label>
            </div>
            <div className='label-control-pair empty-label-margin-right'>
              <Form.Label>Scale</Form.Label>
              <Form.Control step="0.001" min="0" type='number' value={rangefinderScale} onChange={handleRangefinderScaleInputChange} />
              <Form.Label></Form.Label>
            </div>
            <div className='label-control-pair reset-button-pair empty-label-margin-right'>
              <Form.Label className='action-button-label'></Form.Label>
              <Button variant="success" title="Sets a coordinate marker at the current range measurement" onClick={handleRangefinderMarkMap}>Mark Map</Button>
              <Form.Label></Form.Label>
            </div>
            <div className='label-control-pair reset-button-pair empty-label-margin-right'>
              <Form.Label className='action-button-label'></Form.Label>
              <Button variant="primary" title='Moves the rangefinder to the map location' onClick={handleRangefinderJumpToMap}>Jump To Map</Button>
              <Form.Label></Form.Label>
            </div>
            <div className='label-control-pair reset-button-pair empty-label-margin-right'>
              <Form.Label className='reset-button-label action-button-label'></Form.Label>
              <Button variant="danger" title="Resets the parameters of the rangefinder" onClick={handleRangefinderReset}>Reset</Button>
              <Form.Label></Form.Label>
            </div>
            <div className='label-control-pair reset-button-pair empty-label-margin-right'>
              <Form.Label className='reset-button-label action-button-label'></Form.Label>
              <Button variant="warning" title='Resets the rangefinder code objects' onClick={handleRangefinderDebugReset}>Reset Laser</Button>
              <Form.Label></Form.Label>
            </div>
          </div> }
          <div className={"mapBoxGl1-wrapper " + (showMap ? undefined : 'hidden')}>
          <p className='map-title-overlay'>Map View</p>
            <div ref={mapContainerRef} className="mapBox-container-1"></div>
          </div>
          <div className={"mapBoxGl1-wrapper " + (showRanged ? undefined : 'hidden')}>
            <p className='map-title-overlay'>Ranged View</p>
            <div ref={mapContainerRangeRef} className="mapBox-container-2"></div>
          </div>
      </div>
      <Modal show={showHelpModal} onHide={handleCloseHelpModal}>
        <Modal.Header closeButton>
          <Modal.Title>Rangefinder Simulator Help</Modal.Title>
        </Modal.Header>
        <Modal.Body>
          <h4>Program Description</h4>
          <p>
            This program simulates a rangefinder within the 3D space of Mapbox GL. There is a controllable virtual laser chassis that represents the rangefinder hardware; when the virtual laser intersects the map, a corresponding measurement from the front of the rangefinder to the point of intersection is calculated and displayed. The laser can be pointed in three dimensions and the range of the laser can be adjusted.
          </p>
          <h4>Program Operation</h4>
          <p>
            The program consists of a header bar, and a content area containing four main sections:</p>
             <ul>
              <li>the Map controls</li>
              <li>the Rangefinder controls</li>
              <li>the Map view</li>
              <li>the "Ranged View", which is the view centered on the end of the rangefinder's laser</li>
            </ul>
            <p>The display of the four main sections can be toggled on or off via controls in the Settings dropdown of the header bar. The Map View can be controlled by adjusting the Map control parameters or by direct map interaction using the standard Mapbox GL user interface controls. The rangefinder can be controlled by adjusting the Rangefinder control parameters or by directly interacting with the rangefinder; the direct interaction controls for the rangefinder are as follows:</p>
              <ul>
                <li><b>Shift + mouse drag</b>: Adjust the position of the rangefinder</li>
                <li><b>Ctrl + mouse drag</b>: Adjust the altitude of the rangefinder</li>
                <li><b>Alt + mouse drag</b>: Adjust the rotation of the rangefinder</li>
              </ul>
            <p> The Ranged View can also be manipulated similar to the Map View, although it is intended functionally to be automatically centered upon the point of intersection of the rangefinder laser. The Map controls and Rangefinder controls sections each consist of text-editable fields that correspond to orientation options for the map and rangefinder. The Buttons within each section contain functionality to control the position of either the map or rangefinder, as well as reset functions to reset the map and rangefinder to their default parameters; the reset laser button is to be used if the laser disappears from the rangefinder. The Search bar within the Map View is a "geo-coder" that can be used to reposition the map at a specified geographic location.
          </p>
          <h4>Program Limitations</h4>
          <p>
            There are some technical limitations to the program that constrain its functionality. The first limitation is "jitter" that manifests itself in the Map And Ranged Views when adjusting the map and rangefinder; this is likely due to the debouncing mechanism put in place to prevent "stuck" user interface controls when attempting rapid consecutive updates. The second limitation is the nature of the 3D map - the rendered representation of the 3D terrain relies upon Mapbox GL's terrain exaggeration feature, which diverges from the "true" terrain elevation data; as I could not figure out how to reconcile the differences between the two values, the exaggerated terrain is used to visually correspond with the underlying rangefinder distance calculations. The third limiation is the accuracy of the laser measurement, which is bounded by tests of intersection along the length of the laser at each point x/MaxRange, where x is one meter of range.
          </p>
        </Modal.Body>
        <Modal.Footer>
          <Button variant="secondary" onClick={handleCloseHelpModal}>
            Close
          </Button>
        </Modal.Footer>
      </Modal>
    </div>
  );
}