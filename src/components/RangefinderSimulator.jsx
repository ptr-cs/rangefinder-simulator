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

export default function RangefinderSimulator() {
  const mapContainerRef = useRef(null);
  const [map, setMap] = useState(null);
  const [threebox, setThreebox] = useState(null);
  const [rangefinder, setRangefinder] = useState(null);
  const [laser, setLaser] = useState(null);

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

  const setMapCenterDebounced = useRef(_.debounce(setMapCenter, 250));
  const setMapCenterInputDebounced = useRef(_.debounce(setMapCenter, 10));
  const setMapZoomDebounced = useRef(_.debounce(setMapZoom, 250));
  const setMapZoomInputDebounced = useRef(_.debounce(setMapZoom, 10));
  const setMapPitchDebounced = useRef(_.debounce(setMapPitch, 250));
  const setMapPitchInputDebounced = useRef(_.debounce(setMapPitch, 10));
  const setMapBearingDebounced = useRef(_.debounce(setMapBearing, 250));
  const setMapBearingInputDebounced = useRef(_.debounce(setMapBearing, 10));

  mapboxgl.accessToken = process.env.REACT_APP_MAPBOX_TOKEN;

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
    // console.log("startPos")
    // console.log(startPos.x, startPos.y, startPos.z)
    // console.log('endPos')
    // console.log(endPos.x, endPos.y, endPos.z)
    // Create a geometry for the line
    const geometry = new THREE.BufferGeometry().setFromPoints(points);

    // Create a material for the line
    const material = new THREE.LineBasicMaterial({ color: 0xff0000 });

    // Create the line using the geometry and material
    const line = new THREE.Line(geometry, material);
    console.log(line)

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
      initMap.setTerrain({ 'source': 'mapbox-dem', 'exaggeration': 1.5 });

      //   var calcCoords = calculateDestination(37.701857,-122.450648, 1, 1000)
      //   initMap.addSource('lineSource', {
      //     'type': 'geojson',
      //     'data': {
      //         'type': 'Feature',
      //         'properties': {},
      //         'geometry': {
      //             'type': 'LineString',
      //             'coordinates': [
      //                 [-122.450648, 37.701857, 1000], // Start of the line [longitude, latitude]
      //                 [calcCoords.lon, calcCoords.lat, 0]  // End of the line [longitude, latitude]
      //             ]
      //         }
      //     }
      // });

      // // Add a layer to visualize the line
      // initMap.addLayer({
      //     'id': 'lineLayer',
      //     'type': 'line',
      //     'source': 'lineSource',
      //     'layout': {},
      //     'paint': {
      //         'line-width': 8,
      //         'line-color': '#007cbf' // Line color
      //     }
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
            console.log('model')
            console.log(model)
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

    console.log(initMap)
    setMap(initMap);

    return () => initMap.remove();
  }, []);

  function rotateLine(point1, point2, angle) {
    // Convert the angle from degrees to radians
    const angleRadians = THREE.MathUtils.degToRad(angle);

    // Calculate the direction vector from point1 to point2
    const direction = new THREE.Vector3().subVectors(point2, point1);

    // Create a rotation axis (for rotation in the XY plane, the axis is the Z axis)
    const rotationAxis = new THREE.Vector3(0, 0, 1);

    // Create a rotation matrix
    const rotationMatrix = new THREE.Matrix4();
    rotationMatrix.makeRotationAxis(rotationAxis.normalize(), angleRadians);

    // Apply the rotation matrix to the direction vector
    direction.applyMatrix4(rotationMatrix);

    // Update point2 by adding the rotated direction vector to point1
    const rotatedPoint2 = new THREE.Vector3().addVectors(point1, direction);

    return [point1, rotatedPoint2]; // Return the original and the rotated point
  }

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

  // Function to calculate intersection between line and map
  function calculateLineMapIntersection(lineStart, lineEnd, map, threebox) {
    
    // Project 3D line onto the 2D plane of the map
    const start = threebox.unprojectFromWorld(lineStart);
    const end = threebox.unprojectFromWorld(lineEnd);
    // console.log("calculateLineMapIntersection start / end")
    // console.log(start)
    // console.log(end)

    // Create a line segment using projected points
    const lineSegment = [start, end];

    // Get map bounds in screen coordinates
    const bounds = map.getContainer().getBoundingClientRect();
    const mapBounds = {
        min: new THREE.Vector2(bounds.left, bounds.top),
        max: new THREE.Vector2(bounds.right, bounds.bottom)
    };

    // Iterate through each feature in the map
    const features = map.queryRenderedFeatures({ layers: ['add-3d-buildings'] }); // Replace 'your-layer-name' with the name of your map layer
    for (const feature of features) {
        // Get the geometry of the feature
        const geometry = feature.geometry;

        // Iterate through each segment of the geometry
        for (let i = 0; i < geometry.coordinates.length - 1; i++) {
            for (let j = 0; j < geometry.coordinates[i].length - 1; ++j) {
              // Project each segment onto the screen
              const segmentStart = threebox.projectToWorld(geometry.coordinates[i][j]);
              const segmentEnd = threebox.projectToWorld(geometry.coordinates[i][j+1]);

              // Check if the line segment intersects with the projected segment
              const intersection = calculateIntersection(lineSegment[0], lineSegment[1], segmentStart, segmentEnd);
              if (intersection && isPointInsideBoundingBox(intersection, mapBounds)) {
                  // Convert intersection point back to geographical coordinates
                  const lngLat = threebox.unprojectFromWorld(intersection);
                  return { lng: lngLat.lng, lat: lngLat.lat };
              }
            }
        }
    }

    // No intersection found
    return null;
  }

  const handleMapCenterInputChange = (e) => {
    console.log("handleMapCenterInputChange")
    const { name, value } = e.target;
    console.log(e.target)
    if (isNaN(value))
      return
    setMapCenterInputDebounced.current((prevCenter) => ({
      ...prevCenter,
      [name]: parseFloat(value).toFixed(6),
    }));
  };

  const handleMapZoomInputChange = (e) => {
    console.log("handleMapZoomInputChange")
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setMapZoomInputDebounced.current(parseFloat(e.target.value).toFixed(6));
  };

  const handleMapPitchInputChange = (e) => {
    console.log("handleMapPitchInputChange")
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setMapPitchInputDebounced.current(parseFloat(e.target.value).toFixed(6));
  };

  const handleMapBearingInputChange = (e) => {
    console.log("handleMapBearingInputChange")
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setMapBearingInputDebounced.current(parseFloat(e.target.value).toFixed(6));
  };

  const handleRangefinderCoordsInputChangeLat = (e) => {
    console.log("handleRangefinderCoordsInputChange")
    console.log(e)
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setRangefinderCoords([rangefinderCoords[0], Number(parseFloat(e.target.value).toFixed(6)), rangefinderCoords[2]]);
  };

  const handleRangefinderCoordsInputChangeLng = (e) => {
    console.log("handleRangefinderCoordsInputChange")
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setRangefinderCoords([Number(parseFloat(e.target.value).toFixed(6)), rangefinderCoords[1], rangefinderCoords[2]]);
  };

  const handleRangefinderCoordsInputChangeAlt = (e) => {
    console.log("handleRangefinderCoordsInputChange")
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setRangefinderCoords([rangefinderCoords[0], rangefinderCoords[1], Number(parseFloat(e.target.value).toFixed(6))]);
  };

  const handleRangefinderRotationInputChangeX = (e) => {
    console.log("handleRangefinderRotationInputChangeZ")
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setRangefinderRotation({ x: parseFloat(e.target.value).toFixed(6), y: rangefinderRotation.y, z: rangefinderRotation.z });
  };

  const handleRangefinderRotationInputChangeY = (e) => {
    console.log("handleRangefinderRotationInputChangeZ")
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setRangefinderRotation({ x: rangefinderRotation.x, y: parseFloat(e.target.value).toFixed(6), z: rangefinderRotation.z });
  };

  const handleRangefinderRotationInputChangeZ = (e) => {
    console.log("handleRangefinderRotationInputChangeZ")
    console.log(e.target.value)
    if (isNaN(e.target.value))
      return
    setRangefinderRotation({ x: rangefinderRotation.x, y: rangefinderRotation.y, z: parseFloat(e.target.value).toFixed(6) });
  };

  const handleRangefinderRangeInputChange = (e) => {
    console.log("handleRangefinderRangeInputChange")
    console.log(e.target.value)
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
  
  // Function to convert a THREE.Vector3 object to a coordinate object
function vector3ToCoordinate(vector3, map) {
  // Convert 3D point to geographical coordinates (latitude, longitude)
  const lngLat = map.unproject([vector3.x, vector3.y]);
  
  // Get altitude (height above sea level) in meters
  const altitude = vector3.z;

  // Return coordinate object with latitude, longitude, and altitude
  return {
      latitude: lngLat.lat,
      longitude: lngLat.lng,
      altitude: altitude
  };
}

// Function to convert a THREE.BufferGeometry() to coordinates array
function bufferGeometryToCoordinates(bufferGeometry, map) {
  const vertices = bufferGeometry.attributes.position.array;
  const coordinates = [];

  // Iterate over vertices and convert each to coordinates
  for (let i = 0; i < vertices.length; i += 3) {
      const x = vertices[i];
      const y = vertices[i + 1];
      const z = vertices[i + 2];

      // Convert vertex to coordinate object
      const coordinate = vector3ToCoordinate(new THREE.Vector3(x, y, z), map);
      coordinates.push(coordinate);
  }

  return coordinates;
}

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

  useEffect(() => {
    if (rangefinder && laser && rangefinderCoords && rangefinderRotation && rangefinderScale && rangefinderMaxRange) {
      rangefinder.setCoords(rangefinderCoords)
      rangefinder.setRotation(rangefinderRotation)
      updateModelScale(rangefinderScale);
      // Define the start and end coordinates for the laser
      const startCoord = rangefinderCoords
      const destination = calculateDestination(startCoord[1], startCoord[0], 0, rangefinderMaxRange);
      const endCoord = [destination.lon, destination.lat, rangefinderCoords[2]];

      // console.log("pre rot start coord")
      // console.log(startCoord)
      // console.log("post rot start coord")
      // console.log(endCoord)
      // Convert coordinates to Threebox world positions
      const startPos = threebox.projectToWorld(startCoord);
      const endPos = threebox.projectToWorld(endCoord);
      // console.log("threebox projectToWorld")
      // console.log(startPos)
      // console.log(endPos)
      // const startPosMap = map.project(startCoord);
      // const endPosMap = map.project(endCoord);
      // console.log("map projectToWorld")
      // console.log(startPosMap)
      // console.log(endPosMap)
      const points = [];

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
      
      if (laser.geometry) {
        // Get vertices from buffer geometry
        const vertices = getVerticesFromBufferGeometry(laser.geometry);
        console.log(vertices)
        const intersection = calculateLineMapIntersection(vertices[0], vertices[1], map, threebox);
        console.log('Intersection point:', intersection);
      }
      
      
      // console.log("end coords")
      // console.log(endCoord)
      
      // Convert 3D point to geographical coordinates (latitude, longitude)
      // const lngLat = map.unproject([rotatedVertices[0].x, rotatedVertices[0].y]);
      
      // setRangefinderMaxRangeCoordinate([lngLat.lng, lngLat.lat, rotatedVertices[1].z])
      // console.log("max range coordinate")
      // console.log(rangefinderMaxRangeCoordinate)
      //laser.rotation.set(0, 0, 0);
      //laser.rotateOnAxis(new THREE.Vector3(0, 0, 1), THREE.MathUtils.degToRad(rangefinderRotation.z));
      
      //setRangefinderRange()
    }
  }, [rangefinderCoords, rangefinderRotation, rangefinderScale, rangefinderMaxRange]);

  useEffect(() => {
    console.log(debugCounter)
    if (map && !isNaN(mapCenter.lat) && !isNaN(mapCenter.lng) && !isNaN(mapZoom)
      && (prevMapZoom !== parseFloat(mapZoom).toFixed(6) ||
        prevMapCenter.lat !== parseFloat(mapCenter.lat).toFixed(6) ||
        prevMapCenter.lng !== parseFloat(mapCenter.lng).toFixed(6)) ||
      prevMapPitch !== parseFloat(mapPitch).toFixed(6) ||
      prevMapBearing !== parseFloat(mapBearing).toFixed(6)) {
      // console.log("CURRENT:")
      // console.log(mapCenter)
      // console.log(mapZoom)
      // console.log('PREVIOUS')
      // console.log(prevMapCenter)
      // console.log(prevMapZoom)
      // console.log("----------------------")
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