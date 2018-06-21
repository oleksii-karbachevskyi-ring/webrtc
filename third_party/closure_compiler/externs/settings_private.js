// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file was generated by:
//   tools/json_schema_compiler/compiler.py.
// NOTE: The format of types has changed. 'FooType' is now
//   'chrome.settingsPrivate.FooType'.
// Please run the closure compiler before committing changes.
// See https://chromium.googlesource.com/chromium/src/+/master/docs/closure_compilation.md

/** @fileoverview Externs generated from namespace: settingsPrivate */

/**
 * @const
 */
chrome.settingsPrivate = {};

/**
 * @enum {string}
 * @see https://developer.chrome.com/extensions/settingsPrivate#type-PrefType
 */
chrome.settingsPrivate.PrefType = {
  BOOLEAN: 'BOOLEAN',
  NUMBER: 'NUMBER',
  STRING: 'STRING',
  URL: 'URL',
  LIST: 'LIST',
  DICTIONARY: 'DICTIONARY',
};

/**
 * @enum {string}
 * @see https://developer.chrome.com/extensions/settingsPrivate#type-ControlledBy
 */
chrome.settingsPrivate.ControlledBy = {
  DEVICE_POLICY: 'DEVICE_POLICY',
  USER_POLICY: 'USER_POLICY',
  OWNER: 'OWNER',
  PRIMARY_USER: 'PRIMARY_USER',
  EXTENSION: 'EXTENSION',
};

/**
 * @enum {string}
 * @see https://developer.chrome.com/extensions/settingsPrivate#type-Enforcement
 */
chrome.settingsPrivate.Enforcement = {
  ENFORCED: 'ENFORCED',
  RECOMMENDED: 'RECOMMENDED',
};

/**
 * @typedef {{
 *   key: string,
 *   type: !chrome.settingsPrivate.PrefType,
 *   value: *,
 *   controlledBy: (!chrome.settingsPrivate.ControlledBy|undefined),
 *   controlledByName: (string|undefined),
 *   enforcement: (!chrome.settingsPrivate.Enforcement|undefined),
 *   recommendedValue: (*|undefined),
 *   extensionId: (string|undefined),
 *   extensionCanBeDisabled: (boolean|undefined)
 * }}
 * @see https://developer.chrome.com/extensions/settingsPrivate#type-PrefObject
 */
chrome.settingsPrivate.PrefObject;

/**
 * Sets a settings value.
 * @param {string} name The name of the pref.
 * @param {*} value The new value of the pref.
 * @param {string} pageId The user metrics identifier or null.
 * @param {function(boolean):void} callback The callback for whether the pref
 *     was set or not.
 * @see https://developer.chrome.com/extensions/settingsPrivate#method-setPref
 */
chrome.settingsPrivate.setPref = function(name, value, pageId, callback) {};

/**
 * Gets an array of all the prefs.
 * @param {function(!Array<!chrome.settingsPrivate.PrefObject>):void} callback
 * @see https://developer.chrome.com/extensions/settingsPrivate#method-getAllPrefs
 */
chrome.settingsPrivate.getAllPrefs = function(callback) {};

/**
 * Gets the value of a specific pref.
 * @param {string} name
 * @param {function(!chrome.settingsPrivate.PrefObject):void} callback
 * @see https://developer.chrome.com/extensions/settingsPrivate#method-getPref
 */
chrome.settingsPrivate.getPref = function(name, callback) {};

/**
 * Gets the default page zoom factor. Possible values are currently between 0.25
 * and 5. For a full list, see zoom::kPresetZoomFactors.
 * @param {function(number):void} callback
 * @see https://developer.chrome.com/extensions/settingsPrivate#method-getDefaultZoom
 */
chrome.settingsPrivate.getDefaultZoom = function(callback) {};

/**
 * Sets the page zoom factor. Must be less than 0.001 different than a value in
 * zoom::kPresetZoomFactors.
 * @param {number} zoom
 * @param {function(boolean):void=} callback
 * @see https://developer.chrome.com/extensions/settingsPrivate#method-setDefaultZoom
 */
chrome.settingsPrivate.setDefaultZoom = function(zoom, callback) {};

/**
 * <p>Fired when a set of prefs has changed.</p><p>|prefs| The prefs that
 * changed.</p>
 * @type {!ChromeEvent}
 * @see https://developer.chrome.com/extensions/settingsPrivate#event-onPrefsChanged
 */
chrome.settingsPrivate.onPrefsChanged;
