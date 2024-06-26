# Rangefinder simulation: A React and Mapbox GL implementation

Rangefinder simulator.

![rangefinder-simulator-demo](https://github.com/ptr-cs/rangefinder-simulator/assets/112029487/bc1b1294-7848-49a6-b589-7ff401634d0f)

A virtual rangefinder that displays a measurement range on a 3D map is implemented using Mapbox GL, Threebox, and React. The purpose of the app is to calculate the distance and coordinates of a map intersection with an endpoint in 3D space extending a from the 3D position of an observer. Both the map and rangefinder are able to be controlled either via direct manipulation using the mouse or via text inputs bound to position and rotation parameters.

Rangefinder model is courtesy of Kenny.nl 3D assets colleciton: https://kenney.nl/assets?q=3d

![image](https://github.com/ptr-cs/rangefinder-simulator/assets/112029487/2e4770ae-d0df-41a4-807a-912b1e7c749d)

## Installing and launching the application

*Note*: You must use a Mapbox GL access token in order to run this app; once you have obtained an access token, copy and paste the token string in the root of the project in a file called .env like so:

  REACT_APP_MAPBOX_TOKEN="your_access_token_string"

- npm install
- npm run start
  
# License
Educational purposes only. Any other usage requires the express written permission of the repository owner.
