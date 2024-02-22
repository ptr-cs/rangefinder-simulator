# Rangefinder simulation: A React and Mapbox GL implementation

Rangefinder simulator.

A virtual rangefinder that displays a measurement range on a 3D map is implemented using Mapbox GL, Threebox, and React. The purpose of the app is to calculate the distance and coordinates of a map intersection with an endpoint in 3D space extending a from the 3D position of an observer. Both the map and rangefinder are able to be controlled either via direct manipulation using the mouse or via text inputs bound to position and rotation parameters.

Rangefinder model is courtesy of Kenny.nl 3D assets colleciton: https://kenney.nl/assets?q=3d

![rangefinder-simulator-demo3](https://github.com/ptr-cs/rangefinder-simulator/assets/112029487/b8000368-9770-4828-be42-b7e4017dc79c)

![image](https://github.com/ptr-cs/rangefinder-simulator/assets/112029487/50b3714b-e3d3-4766-9e5f-9ca9992c19fe)

## Installing and launching the application

*Note*: You must use a Mapbox GL access token in order to run this app; once you have obtained an access token, copy and paste the token string in the root of the project in a file called .env like so:

  REACT_APP_MAPBOX_TOKEN="your_access_token_string"

- npm install
- npm run start
  
# License
Educational purposes only. Any other usage requires the express written permission of the repository owner.
