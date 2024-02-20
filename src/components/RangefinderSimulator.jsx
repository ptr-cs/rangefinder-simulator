import React, { useRef, useEffect, useState } from 'react';
import Button from 'react-bootstrap/Button';
import Form from 'react-bootstrap/Form';
import './RangefinderSimulator.scss';
import { THREE, Threebox } from "threebox-plugin";
import "threebox-plugin/dist/threebox.css";
import "bootstrap/dist/css/bootstrap.css"
import mapboxgl from '!mapbox-gl'; // eslint-disable-line import/no-webpack-loader-syntax
import { Perf } from 'r3f-perf'
import _, { range } from 'lodash'
import { Vector3 } from 'threebox/src/three64';
import * as turf from '@turf/turf'

export default function RangefinderSimulator() {
  const mapContainerRef = useRef(null);
  const [map, setMap] = useState(null);
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

  const setMapCenterDebounced = useRef(_.debounce(setMapCenter, 250));
  const setMapCenterInputDebounced = useRef(_.debounce(setMapCenter, 10));
  const setMapZoomDebounced = useRef(_.debounce(setMapZoom, 250));
  const setMapZoomInputDebounced = useRef(_.debounce(setMapZoom, 10));
  const setMapPitchDebounced = useRef(_.debounce(setMapPitch, 250));
  const setMapPitchInputDebounced = useRef(_.debounce(setMapPitch, 10));
  const setMapBearingDebounced = useRef(_.debounce(setMapBearing, 250));
  const setMapBearingInputDebounced = useRef(_.debounce(setMapBearing, 10));

  mapboxgl.accessToken = process.env.REACT_APP_MAPBOX_TOKEN;

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

  const onDraggedObject = (e) => {
    setRangefinderCoords(e.detail.draggedObject.coordinates)
    var newRotation = { x: (e.detail.draggedObject.rotation._x * (180 / Math.PI)).toFixed(6), y: (e.detail.draggedObject.rotation._y * (180 / Math.PI)).toFixed(6), z: (e.detail.draggedObject.rotation._z * (180 / Math.PI)).toFixed(6) }
    setRangefinderRotation(newRotation)
  }



  useEffect(() => {
    const initMap = new mapboxgl.Map({
      container: mapContainerRef.current,
      style: 'mapbox://styles/mapbox/streets-v11',
      center: [mapCenter.lng, mapCenter.lat],
      zoom: mapZoom,
      antialias: true // create the gl context with MSAA antialiasing, so custom layers are antialiased
    });

    initMap.addControl(new mapboxgl.NavigationControl());
    initMap.addControl(new mapboxgl.ScaleControl());

    // eslint-disable-next-line no-undef
    const tb = (window.tb = new Threebox(
      initMap,
      initMap.getCanvas().getContext('webgl'),
      {
        defaultLights: true, enableSelectingObjects: true, enableDraggingObjects: true, enableRotatingObjects: true, enableTooltips: true
      }
    ));

    setThreebox(tb)

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

      var calcCoords = calculateDestination(37.701857, -122.450648, 0, rangefinderMaxRange)
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
          const scale = rangefinderScale;
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

    return () => initMap.remove();
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

  // Function to check if a point lies within a bounding box
  function isPointInsideBoundingBox(point, bounds) {
    return point.x >= bounds.min.x && point.x <= bounds.max.x &&
      point.y >= bounds.min.y && point.y <= bounds.max.y;
  }

  // Function to calculate intersection point of two line segments
  function calculateIntersection(lineStart, lineEnd, segmentStart, segmentEnd) {
    const denominator = ((segmentEnd.y - segmentStart.y) * (lineEnd.x - lineStart.x)) - ((segmentEnd.x - segmentStart.x) * (lineEnd.y - lineStart.y));

    // If the two lines are parallel (denominator close to 0), there is no intersection
    if (denominator === 0) {
      return null;
    }

    const ua = (((segmentEnd.x - segmentStart.x) * (lineStart.y - segmentStart.y)) - ((segmentEnd.y - segmentStart.y) * (lineStart.x - segmentStart.x))) / denominator;
    const ub = (((lineEnd.x - lineStart.x) * (lineStart.y - segmentStart.y)) - ((lineEnd.y - lineStart.y) * (lineStart.x - segmentStart.x))) / denominator;

    // If both ua and ub are between 0 and 1, the intersection point is within both line segments
    if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
      const x = lineStart.x + (ua * (lineEnd.x - lineStart.x));
      const y = lineStart.y + (ua * (lineEnd.y - lineStart.y));
      return new THREE.Vector2(x, y);
    }

    return null; // No intersection
  }
  
    /**
   * Clips a 3D line below a certain elevation, making the below-ground part invisible.
   * @param {THREE.BufferGeometry} geometry - The geometry representing the 3D line.
   * @param {number} groundLevel - The elevation at which the ground intersects the line.
   * @returns {THREE.BufferGeometry} A new geometry representing the clipped line.
   */
  function clipLineAtGroundLevel(geometry, groundLevel) {
    const vertices = geometry.getAttribute('position').array;
    const newVertices = [];
    let isAboveGround = vertices[2] > groundLevel;

    for (let i = 0; i < vertices.length; i += 3) {
      // Check if the current point is above the ground
      if (vertices[i + 2] > groundLevel) {
        if (!isAboveGround) {
          // Calculate and add the intersection point with the ground
          // Note: This is a simplification. You'd need to interpolate the actual intersection point.
          newVertices.push(vertices[i], vertices[i + 1], groundLevel);
        }
        newVertices.push(vertices[i], vertices[i + 1], vertices[i + 2]);
        isAboveGround = true;
      } else {
        if (isAboveGround && i > 0) {
          // Calculate and add the intersection point with the ground
          // Note: This is a simplification. You'd need to interpolate the actual intersection point.
          newVertices.push(vertices[i - 3], vertices[i - 2], groundLevel);
        }
        isAboveGround = false;
      }
    }
    // Create a new geometry with the clipped vertices
    const newGeometry = new THREE.BufferGeometry();
    newGeometry.setAttribute('position', new THREE.Float32BufferAttribute(newVertices, 3));
    return newGeometry;
  }
  
  // Function to convert Three.js coordinates to geographic coordinates
  function convertToGeographicCoordinates(threeX, threeY, threeZ, threebox) {
    // Conversion logic here. This will depend on your Three.js and Mapbox setup.
    var lnglat = map.unproject([threeX, threeY, threeZ])
    return { lat: lnglat.lat/* Latitude */, lng: lnglat.lng/* Longitude */, alt: threeZ/* Altitude in meters */ };
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
  
  function findClosestTerrainIntersection(lineInput, map) {
    let intersectionPoints = [];
    let closestIntersection = null;
    let minDistance = Infinity;
  
    // Check if input is BufferGeometry (Three.js line) or simple coordinate array
    if (lineInput instanceof THREE.BufferGeometry) {
      // Handle THREE.BufferGeometry
      const vertices = lineInput.getAttribute('position').array;
      for (let i = 0; i < vertices.length; i += 9) { // Step by 9 for x, y, z of two points
        const start = {lng: vertices[i], lat: vertices[i+1], alt: vertices[i+2]};
        const end = {lng: vertices[i+3], lat: vertices[i+4], alt: vertices[i+5]};
        checkIntersectionAndDistance(start, end, map, intersectionPoints, minDistance, closestIntersection);
      }
    } else {
      // Handle coordinate pair
      const start = {lng: lineInput[0][0], lat: lineInput[0][1], alt: lineInput[0][2]};
      const end = {lng: lineInput[1][0], lat: lineInput[1][1], alt: lineInput[1][2]};
      checkIntersectionAndDistance(start, end, map, intersectionPoints, minDistance, closestIntersection);
    }
  
    // Find closest intersection to the line's origin
    return closestIntersection;
  }
  
  function checkIntersectionAndDistance(start, end, map, intersectionPoints, minDistance, closestIntersection) {
    // Here, implement logic to check each segment for intersection as shown in the original function
    // This is a simplified example that checks just the start point for demonstration
    const terrainElevation = map.queryTerrainElevation([start.lng, start.lat], { exaggerated: true });
    if (start.alt <= terrainElevation) {
      const intersection = { lat: start.lat, lng: start.lng, alt: terrainElevation };
      intersectionPoints.push(intersection);
      
      // Calculate distance from origin to this intersection point
      const distance = Math.sqrt(Math.pow((start.lng - intersection.lng), 2) + Math.pow((start.lat - intersection.lat), 2) + Math.pow((start.alt - intersection.alt), 2));
      if (distance < minDistance) {
        minDistance = distance;
        closestIntersection = intersection;
      }
    }
    // Repeat similar logic for 'end' or other points along the BufferGeometry to find and compare intersections
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

  const handleMapReset = (e) => {
    setMapCenter(MAP_INITIAL_CENTER)
    setMapZoom(MAP_INITIAL_ZOOM)
    setMapPitch(MAP_INITIAL_PITCH)
    setMapBearing(MAP_INITIAL_BEARING)
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

  // Function to calculate the angle between two points in radians
  function calculateAltitudeAngle(point1, point2) {
    // Calculate the altitude difference between the two points
    const altitudeDifference = Math.abs(point2.altitude - point1.altitude);

    // Calculate the distance between the two points in meters
    const lat1 = point1.lat * Math.PI / 180.0;
    const lat2 = point2.lat * Math.PI / 180.0;
    const lon1 = point1.lng * Math.PI / 180.0;
    const lon2 = point2.lng * Math.PI / 180.0;
    const R = 6371000; // Earth radius in meters
    const x = (lon2 - lon1) * Math.cos((lat1 + lat2) / 2);
    const y = (lat2 - lat1);
    const distance = Math.sqrt(x * x + y * y) * R;

    // Calculate the angle of altitude difference using trigonometry
    const angleRadians = Math.atan(altitudeDifference / distance);

    // Convert radians to degrees
    const angleDegrees = angleRadians * 180.0 / Math.PI;

    return angleDegrees;
  }

  // Function to calculate the intersection point of line B through A
  function calculateIntersectionPoint(angleB, yOffsetB, length) {
    // Convert angle from degrees to radians
    const thetaRadians = angleB * (Math.PI / 180);
    // Calculate the Y-coordinate of line A (same as starting position of line B on Y-axis)
    const yA = yOffsetB;
    // Calculate the X-coordinate of the intersection point on line B
    const xIntersection = yA / Math.tan(thetaRadians);
    return { x: xIntersection, y: 0 };
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
          setRangefinderRange(NaN.toString())
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

      laser.geometry.setFromPoints(rotatedVertices);

      // Get vertices from buffer geometry
      const verticesFromGeometry = getVerticesFromBufferGeometry(laser.geometry);

      const laserStart = threebox.unprojectFromWorld(verticesFromGeometry[0]);
      const laserEnd = threebox.unprojectFromWorld(verticesFromGeometry[1]);
      console.log("laserStart")
      console.log(laserStart)
      updateLinePosition([[laserStart[0], laserStart[1]], [laserEnd[0], laserEnd[1]]])
      var thetaAngleDegrees = calculateAltitudeAngle({ lng: laserStart[0], lat: laserStart[1], altitude: laserStart[2] }, { lng: laserEnd[0], lat: laserEnd[1], altitude: laserEnd[2] })
      var intersection = calculateIntersectionPoint(thetaAngleDegrees, rangefinderCoords[2], rangefinderRange)
      const sideA = Math.pow(intersection.x, 2)
      const sideB = Math.pow(rangefinderCoords[2], 2)
      const hypotenuse = Math.sqrt(sideA + sideB)
      const roundedHypotenuse = parseFloat(hypotenuse).toFixed(6)
      
      setInterpolatedIntersectionDebounced.current(laser, verticesFromGeometry[0], verticesFromGeometry[1], laserStart, laserEnd, rangefinderMaxRange, map, threebox)
      
    }
  }, [rangefinderCoords, rangefinderRotation, rangefinderScale, rangefinderMaxRange]);

  useEffect(() => {
    if (map && lineSourceData) {
      try {
        map.getSource('lineSource').setData(lineSourceData);
      } catch { }

    }
  }, [lineSourceData]);

  useEffect(() => {
    if (map && !isNaN(mapCenter.lat) && !isNaN(mapCenter.lng) && !isNaN(mapZoom)
      && (prevMapZoom !== parseFloat(mapZoom).toFixed(6) ||
        prevMapCenter.lat !== parseFloat(mapCenter.lat).toFixed(6) ||
        prevMapCenter.lng !== parseFloat(mapCenter.lng).toFixed(6)) ||
        prevMapPitch !== parseFloat(mapPitch).toFixed(6) ||
        prevMapBearing !== parseFloat(mapBearing).toFixed(6)) {
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
  }, [mapCenter, mapZoom, mapPitch, mapBearing, map]);

  return (
    <div className='parent-div'>
      <div className="sim-controls ">
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
          <Form.Label>ꭩ</Form.Label>
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
          <Form.Label className='reset-button-label'></Form.Label>
          <Button variant="primary" onClick={handleMapReset}>Reset</Button>
          <Form.Label></Form.Label>
        </div>
      </div>
      <div className="sim-controls ">
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
          <Form.Label>ꭩ</Form.Label>
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
          <Form.Label>ꭩ</Form.Label>
        </div>
        <div className='label-control-pair altitude-pair'>
          <Form.Label>Range</Form.Label>
          <Form.Control step="0.000001" type='number' value={rangefinderRange} readOnly={true} />
          <Form.Label>ꭩ</Form.Label>
        </div>
        <div className='label-control-pair empty-label-margin-right'>
          <Form.Label>Scale</Form.Label>
          <Form.Control step="0.001" min="0" type='number' value={rangefinderScale} onChange={handleRangefinderScaleInputChange} />
          <Form.Label></Form.Label>
        </div>
        <div className='label-control-pair reset-button-pair empty-label-margin-right'>
          <Form.Label className='reset-button-label'></Form.Label>
          <Button variant="primary" onClick={handleRangefinderReset}>Reset</Button>
          <Form.Label></Form.Label>
        </div>
      </div>
      <div className='mapBoxGl1-wrapper'>
        <div ref={mapContainerRef} className="mapBox-container-1"></div>
      </div>
    </div>

  );
}