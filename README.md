
<h1>UnitySoftBodies</h1>

## Overview
Simple script that automatically connects your SkinnedMeshRenderer bones with joints

<img src="https://media4.giphy.com/media/v1.Y2lkPTc5MGI3NjExcGQzajJwaHN4OGdubTZtZTVtaTIyN3diOWdwc2VqNjNxZXNxNzhleiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/bboyHpyzCUveo3wbLr/giphy.gif" width="100%" height="100%"/>

## Instructions
<ol>
  <li>Download and copy the Scripts and _Demo (optional) folders into an empty folder inside your Assets folder</li>
  <li>Make sure your mesh is properly rigged (The python scripts inside the BlenderAddons directory can help with rigging)</li>
  <li>Add a collider (Any type) to all the bones of your SkinnedMeshRenderer</li>
  <li>Add the SoftBody script to the gameobject that has the SkinnedMeshRenderer component and press Setup SoftBody</li>
</ol>

## Technical Details
The bones are connected by first storing all vertices each bone is influencing in a Dictorary.
A and B is then connected if myDictorary[B] contains any vertex in myDictorary[A].

## License
The code and assets created for this project are licensed under MIT - See the `LICENSE` file for more details.

