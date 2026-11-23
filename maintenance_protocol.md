# Model and firmware maintenance RPC

`ServiceId::kModelMaintenance` is service **10**. Method IDs are declared in
`cpp/include/tcp_data_service_ids.h` and are append-only. All bodies are UTF-8
JSON objects carried by existing authenticated TCP v3 RPC frames. UDP motion
layouts and the TCP framing version are unchanged.

## Methods

| ID | Method | Request | Successful response |
| --- | --- | --- | --- |
| 1 | RequestPermission | `{}` | Model permission `state` |
| 2 | GetPermission | `{}` | Model permission `state` |
| 3 | PrepareActivation | `version_id`, `expected_active`, `request_id` | Operation `request_id`, `status` |
| 4 | GetOperation | `request_id` | Operation status |
| 5 | GetCalibrationBinding | `{}` | `binding_sha256`, `binding`, `worker_ready`, `reason`, deployment details |
| 6 | UploadCalibration | `deployment_id`, `txt`, `zero_convention_id`, `binding_sha256` | Deployment `request_id`, `status` |
| 7 | GetCalibrationDeployment | `deployment_id` | Deployment status and candidate/acceptance details |
| 8 | CommitCalibrationDeployment | `deployment_id`, `acceptance` object, `expected_active`, `request_id` | Operation `request_id`, `status` |
| 9 | DiscardCalibrationDeployment | `deployment_id` | Deployment status |
| 10 | StageFirmware | `job_id` | Firmware `job_id`, `state` |
| 11 | QueueFirmwareForRestart | `job_id` | Firmware `job_id`, `state`, optional `message` |
| 12 | GetFirmwareJob | `job_id` | Firmware `job_id`, `state`, job details |
| 13 | RequestFirmwarePermission | `{}` | Firmware permission `state` |
| 14 | GetFirmwarePermission | `{}` | Firmware permission `state` |

IDs are canonical UUIDv4 strings; `expected_active` is a version UUID or JSON
`null` for factory. The response field `request_id` on deployment queries refers
to `deployment_id`, not the independent commit request UUID. SHA256 fields are
64 lowercase hexadecimal characters. Clients preserve additional response
details and must not infer success from a merely accepted/queued request.

## Transfer and lifecycle

Calibration TXT is embedded in method 6, limited to 32 KiB before JSON escaping.
Methods 6 and 8 accept at most 60 KiB of JSON; the other maintenance requests
accept at most 2048 bytes. The original nominal-model binding, verified zero
convention and independent acceptance remain required. Upload and validation do
not switch the active model. Typical deployment states include `queued`,
`importing`, `building`, `validating`, `awaiting_acceptance`, `pending_restart`
and failure/interruption states. A successful pending commit takes effect after
a controlled restart and must be checked against the actually loaded version.

Firmware uses existing authenticated blob upload with the name
`<job_id>.tar.gz`, followed by methods 10 and 11. SDK chunks are 60 KiB to leave
space inside the 64 KiB transport frame for headers and authentication. The
controller accepts an archive up to 512 MiB; archive/compatibility validation is
separate from transport success. No install or restart occurs as a side effect
of blob upload or staging. States include `validated`, `pending_restart`,
`installing`, `installed`, `failed` and `interrupted`. Query the same job ID after
timeouts or reconnects; `pending_restart` does not mean the new firmware is
currently running.

Model methods 1/2 and firmware methods 13/14 use **distinct**, session-bound
physical grants. A short button press must follow the request within 30 seconds;
the grant lasts 60 seconds and is consumed on submission. Model activation
requires stopped/disabled hardware; firmware permission allows verified
stationary Idle. Neither grant replaces the controller's hardware eligibility
checks. Firmware installation is handled on the next managed service startup.

## Compatibility and errors

Use the existing RPC capability advertisement to discover supported methods.
Older controllers may advertise only methods 1–4 or 1–9, or no maintenance
service. An unsupported method may return 404/501; an SDK must report unsupported
functionality rather than fall back to another numeric method. A 404 from a job
query may also mean that job is unavailable; capability information distinguishes
that case. Nonzero RPC status is failure even if the body is valid JSON.

Typical statuses are 400/413 for invalid or oversized input, 401 for invalid
session, 403 for missing physical permission, 409 for state/conflict/admission
failure, and 503 for unavailable prerequisites. Error bodies may contain a
`reason` string. Preserve that reason. Do not automatically restart, clear a
Fault, or generate a new job/request UUID after an uncertain commit result.
