% Function to update parameter values in the example
% ElectricVehicleThermalManagementWithHeatPump for the specified scenario.

% Copyright 2024 The MathWorks, Inc.

function ElectricVehicleThermalManagementWithHeatPumpScenario(scenario)

% Scenario subsystem block path
subsys = [bdroot '/Scenario'];

% Handle to model workspace
mws = get_param(bdroot, 'ModelWorkspace');

switch scenario
    case 'drive cycle'
        % Switch signal editor group
        set_param([subsys '/Vehicle Speed [km//hr]'], 'ActiveScenario', 'DriveCycle') % Drive cycle

        % Switch temperature source
        set_param([subsys '/Variant Source'], 'LabelModeActiveChoice', 'constant') % Constant temperature

        % Update constant block values
        set_param([subsys '/Pressure [MPa]'], 'Value', '0.101325') % [MPa] Environment pressure
        set_param([subsys '/Temperature [degC]'], 'Value', '30') % [degC] Environment temperature
        set_param([subsys '/Relative Humidity'], 'Value', '0.4') % Environment relative humidity
        set_param([subsys '/CO2 Fraction'], 'Value', '4e-4') % Environment CO2 mole fraction

        set_param([subsys '/Desired Temperature [degC]'], 'Value', '21') % [degC] Desired cabin temperature
        set_param([subsys '/Number of Occupants'], 'Value', '1') % Number of occupants in cabin

        % Update model workspace parameter values
        assignin(mws, 'cabin_p_init', 0.101325) % [MPa] Initial air pressure
        assignin(mws, 'cabin_T_init', 30) % [degC] Initial air temperature
        assignin(mws, 'cabin_RH_init', 0.4) % Initial relative humidity
        assignin(mws, 'cabin_CO2_init', 4e-4) % Initial CO2 mole fraction

        assignin(mws, 'coolant_p_init', 0.101325) % [MPa] Initial coolant pressure
        assignin(mws, 'coolant_T_init', 30) % [degC] Initial coolant temperature

        assignin(mws, 'refrigerant_p_init', 0.8) % [MPa] Initial refrigerant pressure
        assignin(mws, 'refrigerant_alpha_init', 0.6) % Initial refrigerant vapor void fraction

        assignin(mws, 'battery_T_init', 30) % [degC] Initial battery temperature
        assignin(mws, 'battery_Qe_init', 0) % [A*hr] Initial charge deficit

    case 'cool down'
        % Switch signal editor group
        set_param([subsys '/Vehicle Speed [km//hr]'], 'ActiveScenario', 'ZeroSpeed') % Zero speed

        % Switch temperature source
        set_param([subsys '/Variant Source'], 'LabelModeActiveChoice', 'constant') % Constant temperature

        % Update constant block values
        set_param([subsys '/Pressure [MPa]'], 'Value', '0.101325') % [MPa] Environment pressure
        set_param([subsys '/Temperature [degC]'], 'Value', '40') % [degC] Environment temperature
        set_param([subsys '/Relative Humidity'], 'Value', '0.5') % Environment relative humidity
        set_param([subsys '/CO2 Fraction'], 'Value', '4e-4') % Environment CO2 mole fraction

        set_param([subsys '/Desired Temperature [degC]'], 'Value', '21') % [degC] Desired cabin temperature
        set_param([subsys '/Number of Occupants'], 'Value', '1') % Number of occupants in cabin

        % Update model workspace parameter values
        assignin(mws, 'cabin_p_init', 0.101325) % [MPa] Initial air pressure
        assignin(mws, 'cabin_T_init', 40) % [degC] Initial air temperature
        assignin(mws, 'cabin_RH_init', 0.5) % Initial relative humidity
        assignin(mws, 'cabin_CO2_init', 4e-4) % Initial CO2 mole fraction

        assignin(mws, 'coolant_p_init', 0.101325) % [MPa] Initial coolant pressure
        assignin(mws, 'coolant_T_init', 40) % [degC] Initial coolant temperature

        assignin(mws, 'refrigerant_p_init', 1) % [MPa] Initial refrigerant pressure
        assignin(mws, 'refrigerant_alpha_init', 0.76) % Initial refrigerant vapor void fraction

        assignin(mws, 'battery_T_init', 40) % [degC] Initial battery temperature
        assignin(mws, 'battery_Qe_init', 0) % [A*hr] Initial charge deficit

    case 'cold weather'
        % Switch signal editor group
        set_param([subsys '/Vehicle Speed [km//hr]'], 'ActiveScenario', 'DriveCycle') % Drive cycle

        % Switch temperature source
        set_param([subsys '/Variant Source'], 'LabelModeActiveChoice', 'constant') % Constant temperature

        % Update constant block values
        set_param([subsys '/Pressure [MPa]'], 'Value', '0.101325') % [MPa] Environment pressure
        set_param([subsys '/Temperature [degC]'], 'Value', '-10') % [degC] Environment temperature
        set_param([subsys '/Relative Humidity'], 'Value', '0.5') % Environment relative humidity
        set_param([subsys '/CO2 Fraction'], 'Value', '4e-4') % Environment CO2 mole fraction

        set_param([subsys '/Desired Temperature [degC]'], 'Value', '21') % [degC] Desired cabin temperature
        set_param([subsys '/Number of Occupants'], 'Value', '1') % Number of occupants in cabin

        % Update model workspace parameter values
        assignin(mws, 'cabin_p_init', 0.101325) % [MPa] Initial air pressure
        assignin(mws, 'cabin_T_init', -10) % [degC] Initial air temperature
        assignin(mws, 'cabin_RH_init', 0.5) % Initial relative humidity
        assignin(mws, 'cabin_CO2_init', 4e-4) % Initial CO2 mole fraction

        assignin(mws, 'coolant_p_init', 0.101325) % [MPa] Initial coolant pressure
        assignin(mws, 'coolant_T_init', -10) % [degC] Initial coolant temperature

        assignin(mws, 'refrigerant_p_init', 0.2) % [MPa] Initial refrigerant pressure
        assignin(mws, 'refrigerant_alpha_init', 0.60) % Initial refrigerant vapor void fraction

        assignin(mws, 'battery_T_init', -10) % [degC] Initial battery temperature
        assignin(mws, 'battery_Qe_init', 0) % [A*hr] Initial charge deficit

    case 'mid season'

        % Switch signal editor group
        set_param([subsys '/Vehicle Speed [km//hr]'], 'ActiveScenario', 'DriveCycle') % Drive cycle

        % Switch temperature source
        set_param([subsys '/Variant Source'], 'LabelModeActiveChoice', 'variable') % Variable temperature

        % Update constant block values
        set_param([subsys '/Pressure [MPa]'], 'Value', '0.101325') % [MPa] Environment pressure
        set_param([subsys '/Temperature [degC]'], 'Value', '25') % [degC] Environment temperature
        set_param([subsys '/Relative Humidity'], 'Value', '0.4') % Environment relative humidity
        set_param([subsys '/CO2 Fraction'], 'Value', '4e-4') % Environment CO2 mole fraction

        set_param([subsys '/Desired Temperature [degC]'], 'Value', '21') % [degC] Desired cabin temperature
        set_param([subsys '/Number of Occupants'], 'Value', '1') % Number of occupants in cabin

        % Update model workspace parameter values
        assignin(mws, 'cabin_p_init', 0.101325) % [MPa] Initial air pressure
        assignin(mws, 'cabin_T_init', 25) % [degC] Initial air temperature
        assignin(mws, 'cabin_RH_init', 0.4) % Initial relative humidity
        assignin(mws, 'cabin_CO2_init', 4e-4) % Initial CO2 mole fraction

        assignin(mws, 'coolant_p_init', 0.101325) % [MPa] Initial coolant pressure
        assignin(mws, 'coolant_T_init', 25) % [degC] Initial coolant temperature

        assignin(mws, 'refrigerant_p_init', 0.8) % [MPa] Initial refrigerant pressure
        assignin(mws, 'refrigerant_alpha_init', 0.6) % Initial refrigerant vapor void fraction

        assignin(mws, 'battery_T_init', 25) % [degC] Initial battery temperature
        assignin(mws, 'battery_Qe_init', 0) % [A*hr] Initial charge deficit

    otherwise
        error('Invalid scenario')
end

% Update Scenario subsystem tag for annotation
set_param(subsys, 'Tag', scenario)

end