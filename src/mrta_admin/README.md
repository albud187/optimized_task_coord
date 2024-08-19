This `mrta_admin` package can be considered to be the "entry point" for executing multiple robot task allocation scenarios. It contains the following subdirectories:

 `launch` - contains the launch file to execute the task allocation algorithm in the `mrta_main` package

 `scenarios` - contains scenario descriptions. To create a new scenario, create a new subdirectory within the scenarios directory:

  `cd src/mrta_admin/scenarios`

   `mkdir new_scenario`

   Within  `src/mrta_admin/scenarios/new_scenario`, you will need to create the following files:

  You can follow the included within `src/mrta_admin/scenarios/31T-6R`
  
  `tasks.csv`

  `agents.csv`

  `suitabilities.csv`

  
   
 
