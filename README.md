# Image_processing
## Perception step
- Define Source,Destination points for perspective transformation
- Apply Perspective Transform --> Warped 
- Apply thresholding 
  - identify Navigable terrian
  - identify obstacles
  - identify Rock saples
- Update Vision image (dispalyed on left side)
  - channel 0: obstacles threshold *255
  - channel 1: rock sample threshold*255
  - channel 2: nav-terrian threshold *255
- Convert image pixels to Rover-centric coordinates for nav-terrain,obstacles,rocks
- Convert Rover-centric Coordinates to world map coordinates
  - Rotation
  - Translation with Scale
- convert Rover-centric Coordinates to polar coordiantes to get Angles,distances for nav-terrain,obstacles,rocks
- update Rover world map (dispalyed on Right side)
  - channel 0: obstacles 
  - channel 1: rock sample 
  - channel 2: nav-terrian 
- Clear low certainty nav-terian pixels perodically(channel 2)

## Decision step
can be divided into two parts<br>
1- Dealing with normal states<br>
2- Dealing with special states
#### Let's imagine a state diagram with the folling states:
    - Normal states like ['forward movement', 'stop'].
    - Other special states like ['sample detected', 'stuck in a loop', 'stuck by an obstacle', etc...]
    
    
