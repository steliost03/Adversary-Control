%{
Copyright (C) 2017 Stylianos Tsiakalos
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
%}

%%Adversary Control Simulation %%

clear all;
close all;
clc;
warning off;

addpath(genpath('../Adversary_Control/functions'));
addpath(genpath('../Adversary_Control/tbxmanager'));
addpath(genpath('../Adversary_Control/attack_types/regular_attack'));
addpath(genpath('../Adversary_Control/attack_types/covert_attack'));

disp 'Adversary Control Simulation'

game_mode = 0;
while(not(game_mode == 1) && not(game_mode == 2))
    disp 'Select attacker type : ';
    disp '1.Regular'
    disp '2.Covert'
    game_mode = input('');
end

if(game_mode == 1)
    REG_ADVERSARY_CONTROL();
else
    COVERT_ADVERSARY_CONTROL();
end
