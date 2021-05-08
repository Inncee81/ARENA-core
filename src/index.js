/**
 * @fileoverview Load ARENA source and dependencies
 *
 * Open source software under the terms in /LICENSE
 * Copyright (c) 2020, The CONIX Research Center. All rights reserved.
 * @date 2020
 */

// load order: AFRAME, ARENA, components that depend on AFRAME and ARENA
import 'aframe'; // AFRAME
import './arena.js'; // ARENA
import './ui/'; // 2D UI
import './aframe-mods/'; // AFRAME modifications
import './components/'; // custom AFRAME components
import './webar/';
