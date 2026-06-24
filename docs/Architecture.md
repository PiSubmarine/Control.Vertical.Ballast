# Control.Vertical.Ballast

`PiSubmarine.Control.Vertical.Ballast` implements vertical control using ballast fill.

## Responsibility

`Controller` accepts vertical commands and:

- forwards direct ballast-position requests to `Ballast::Api::IController`
- snapshots current depth for `KeepCurrent`
- runs an outer depth PID controller for depth-related modes
- commands empty ballast as a catastrophe fail-safe when depth telemetry is unavailable

The outer loop produces ballast fill targets and delegates actual piston motion to `Ballast::Api::IController`.

## Non-responsibilities

This module does not define:

- the inner ballast-position loop
- depth telemetry transport
- operator input transport
