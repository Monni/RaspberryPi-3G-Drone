
        
            // get the canvas DOM element
            var canvas = document.getElementById('renderCanvas');

            // load the 3D engine
            var engine = new BABYLON.Engine(canvas, true);

            // createScene function that creates and return the scene
            var createScene = function(){
                // create a basic BJS Scene object
                var scene = new BABYLON.Scene(engine);
                // create a FreeCamera
                var camera = new BABYLON.FreeCamera('camera1', new BABYLON.Vector3(0, 5,-10), scene);
                // target the camera to scene origin
                camera.setTarget(BABYLON.Vector3.Zero());
                // create a basic light, aiming 0,1,0 - meaning, to the sky
                var light = new BABYLON.HemisphericLight('light1', new BABYLON.Vector3(0,1,0), scene);

    //Creation of 3 boxes and 2 spheres
    var box1 = BABYLON.Mesh.CreateBox("Box1", 0.0, scene);
    //Moving boxes on the x axis
    box1.position.x = 0;
    //Rotate box around the x axis
    box1.rotation.x = 0;
	box1.rotation.y = 0;

                // return the created scene
                return scene;
            }

            // call the createScene function
            var scene = createScene();

            // run the render loop
            engine.runRenderLoop(function(){
                scene.render();
            });

            // the canvas/window resize event handler
            window.addEventListener('resize', function(){
                engine.resize();
            });
       
