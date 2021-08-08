{
	// here we define a variable for record keeping
	teamFitness : 0,	
	// function called at the beginning of each simulation
	setupSimulation: function() {
            //gathering zone aabb
            this.gMinX = Number.MIN_VALUE;
            this.gMaxX = Number.MAX_VALUE;
            this.gMinY = Number.MIN_VALUE;
            this.gMaxY = Number.MAX_VALUE;
            this.gMinZ = Number.MIN_VALUE;
            this.gMaxZ = Number.MAX_VALUE;
            this.getEnvironment().getGatheringZone().getAABB(
                                                            gMinX, 
                                                            gMaxX, 
                                                            gMinY,
                                                            gMaxY, 
                                                            gMinZ, 
                                                            gMaxZ
                                                        );
            return true;
	},

        afterSimulationStep: function(){
            //resource aabb
            resources = this.getEnvironment().getResources();
            for (var i = 0; i < resources.length; i++) {
                if(!resources[i].isCollected()){
                    var rMinX = Number.MIN_VALUE;
                    var rMaxX = Number.MAX_VALUE;
                    var rMinY = Number.MIN_VALUE;
                    var rMaxY = Number.MAX_VALUE;
                    var rMinZ = Number.MIN_VALUE;
                    var rMaxZ = Number.MAX_VALUE;
                    resources[i].getAABB(
                                        rMinX, 
                                        rMaxX, 
                                        rMinY, 
                                        rMaxY, 
                                        rMinZ, 
                                        rMaxZ
                                        );
                    var inZoneX = false;
                    var inZoneY = false;
                    if (rMinX >= this.gMinX || rMaxX <= this.gMaxX){
                        inZoneX = true;
                    }
                    if (rMinY >= this.gMinY || rMaxY <= this.gMaxY){
                        inZoneY = true;
                    }
                    if(inZoneX && inZoneY){
                        if (this.getEnvironment().getGatheringZone().addResource(resource[i])){
                            value = resource[i].getValue();
                            this.teamFitness += value;
                        }
                        
                    }
                }
                
            }
            
            return true;
        }
	// function called at the end of each simulation
	endSimulation: function() {
            return true;
	},


	// here we return minimum distance travelled across evaluations
	getFitness: function() {
            stepsTaken = 10000;
            maxSteps = 5 //trial evaluatoins per generation
                       * stepsTaken;
            fitness = (this.teamFitness / totalResourceValue) * 100 
                    + (stepsTaken / maxSteps) * 20;
            }
            return fitness;
	},

}