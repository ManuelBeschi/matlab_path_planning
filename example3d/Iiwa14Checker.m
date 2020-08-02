classdef Iiwa14Checker<CollisionChecker
    
    properties
    iiwa;
    collisionArrayFromMesh;
    iiwaCollision;
    cube;
    end
    
    methods
        function r = init(obj)

                init@CollisionChecker(obj,'example',1);
                %World obstacles
                obs_size1=0.5;
                obs_size2=0.5;
                obs_size3=0.8;
                obj.cube=collisionBox(obs_size1,obs_size2,obs_size3);
                T=eye(4);
                T(1,4)=0.55;
                T(2,4)=0.85;
                T(3,4)= obs_size3/2;
                obj.cube.Pose=T;
                
                obj.iiwa = importrobot('iiwa14.urdf');
                obj.iiwa.DataFormat = 'column';
                obj.iiwa.Gravity = [0, 0, 0];
                
                obj.collisionArrayFromMesh = cell(obj.iiwa.NumBodies+1, 2);
                
                collisionMeshPath = fullfile(matlabroot, 'toolbox', 'robotics', ...
                    'robotexamples', 'robotmanip', 'data', 'iiwa_description', ...
                    'meshes', 'iiwa14', 'collision');
                obj.iiwaCollision = importrobot('iiwa14.urdf','MeshPath', collisionMeshPath);
                obj.iiwaCollision.DataFormat = 'column';
                
                
                robotBodies = [{obj.iiwaCollision.Base} obj.iiwaCollision.Bodies];
                for i = 1:numel(robotBodies)
                    if ~isempty(robotBodies{i}.Visuals)
                        % Assumes the first Visuals element is the correct one.
                        visualDetails = robotBodies{i}.Visuals{1};
                        
                        % Extract the part of the visual that actually specifies the STL name
                        visualParts = strsplit(visualDetails, ':');
                        stlFileName = visualParts{2};
                        
                        % Read the STL file
                        stlData = stlread(fullfile(collisionMeshPath, stlFileName));
                        
                        % Create a collisionMesh object from the vertices
                        obj.collisionArrayFromMesh{i,1} = collisionMesh(stlData.Points);
                        
                        % Transform is always identity
                        obj.collisionArrayFromMesh{i,2} = eye(4);
                    end
                end
        end
        
        function r = check(obj,config)
            
            tree = obj.iiwaCollision;
            bodyCollisionArray = obj.collisionArrayFromMesh;
            worldCollisionArray = obj.cube;
            isExhaustiveChecking = false;
            
            % Basic validation
            validateattributes(tree, {'robotics.RigidBodyTree'}, {'nonempty'}, 'exampleHelperManipSelfCollisions', 'tree');
            validateattributes(bodyCollisionArray, {'cell'}, {'nonempty','nrows',tree.NumBodies+1, 'ncols', 2}, 'exampleHelperManipSelfCollisions', 'tree');
            validateattributes(config, {'double'}, {'nonempty','vector','nrows',length(tree.homeConfiguration)}, 'exampleHelperManipSelfCollisions', 'config');
            validateattributes(isExhaustiveChecking, {'logical'}, {'nonempty','scalar'}, 'exampleHelperManipSelfCollisions', 'isExhaustiveChecking');
            
            % Initialize the outputs
            isInCollision = false;
            selfCollisionPairIdx = [];
            worldCollisionPairIdx = [];
            
            % Initialize key parameters
            tree.DataFormat = 'column';
            robotBodies = [{tree.Base} tree.Bodies];
            
            % Rather than calling getTransform at each loop, populate a transform
            % tree, which is a cell array of all body transforms with respect to
            % the base frame
            transformTree = cell(numel(robotBodies),1);
            
            % For the base, this is the identity
            transformTree{1} = eye(4);
            for i = 1:numel(robotBodies)
                transformTree{i} = getTransform(tree, config, robotBodies{i}.Name);
            end
            
            % Iterate over all bodies
            for j = 1:numel(robotBodies)
                % Since collisionPairIdx order doesn't matter, only check every
                % pair of bodies once
                for k = j:numel(robotBodies)
                    % Don't check collision with self or neighbors
                    if j ~= k && j ~= k+1 && j ~= k-1 % not checking with self and neighbors
                        % Ensure that both bodies have associated collision objects
                        if ~isempty(bodyCollisionArray{j,1}) && ~isempty(bodyCollisionArray{k,1})
                            
                            % Get the collision object pose from the associated
                            % rigid body tree. The updated pose is the product of
                            % the associated rigid body tree pose and transform
                            % that relates the collision object origin to the rigid
                            % body position (measured from the parent joint).
                            bodyCollisionArray{j,1}.Pose = transformTree{j}*bodyCollisionArray{j,2};
                            bodyCollisionArray{k,1}.Pose = transformTree{k}*bodyCollisionArray{k,2};
                            
                            % Check for local collision and update the overall
                            % collision status flag
                            localCollisionStatus = checkCollision(bodyCollisionArray{j}, bodyCollisionArray{k});
                            isInCollision = isInCollision || localCollisionStatus;
                            
                            % If a collision is detected, update the matrix of bodies
                            % in collision
                            if localCollisionStatus
                                selfCollisionPairIdx = [selfCollisionPairIdx; [j k]]; %#ok<AGROW>
                                
                                if ~isExhaustiveChecking
                                    r = 0;
                                    return;
                                end
                            end
                        end
                    end
                end
                
                % Check collisions with all the world collision objects
                for m = 1:numel(worldCollisionArray)
                    if ~isempty(bodyCollisionArray{j,1})
                        % The body poses have already been updated in the previous
                        % for-loop, and the world collision objects don't move, so
                        % there's no need to update poses
                        
                        % Check for local collision and update the overall
                        % collision status flag
                        localCollisionStatus = checkCollision(bodyCollisionArray{j}, worldCollisionArray(m));
                        isInCollision = isInCollision || localCollisionStatus;
                        
                        % If a collision is detected, update the matrix of bodies
                        % in collision
                        if localCollisionStatus
                            worldCollisionPairIdx = [worldCollisionPairIdx; [j m]]; %#ok<AGROW>
                            
                            if ~isExhaustiveChecking
                                r = 0;
                                return;
                            end
                        end
                    end
                end
            end
            
            r = ~isInCollision;
        end
        
        function plot3d(obj,configuration)

            show(obj.iiwa,configuration);
            hold on
            show(obj.cube)
            hold off
            
        end
         
        
        function plot(obj)
            
        end
    end
end