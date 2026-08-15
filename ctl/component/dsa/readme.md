# CTL DSA and Data Link Scope

`dsa_dl_scope` is the standard bridge between CTL data acquisition and the
desktop Data Link Scope. This relationship is intentional and is declared in
the facility registry: the adapter depends on the generic Data Link `scope`
facility, `dsa_trigger`, and `dsa_scope`.

The adapter owns the trigger, recorder, pre-trigger history, generation,
capture state, trigger position, mode, level, timeout, and sampling divider.
Applications must not duplicate these fields or implement private
`arm/configure/status/history` callbacks. Trigger configuration is a host-side
concern. A newly initialized adapter defaults to rising-edge triggering on
channel 0 at level 0, with a 50% trigger position and a 1000 ms auto timeout.

## Application contract

The application provides one workspace, the Scope command ID, channel count,
sample rate, and optional display name. The workspace contains both the
published snapshot and private pre-trigger history:

```c
#define SAMPLE_DEPTH 400U

ctrl_gt scope_workspace[
    CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(2U, SAMPLE_DEPTH)];
ctl_dsa_dl_scope_t scope;

gmp_dev_dl_init(&datalink);
ctl_init_dsa_dl_scope_workspace(
    &scope, &datalink, 0x60U, "Control Scope",
    scope_workspace,
    sizeof(scope_workspace) / sizeof(scope_workspace[0]),
    2U, 1000U);
gmp_dev_dl_append_facility(&datalink,
                           ctl_dsa_dl_scope_facility(&scope));
```

Feed samples from the deterministic control interrupt with
`ctl_step_dsa_dl_scope()` or its two/four-channel convenience wrappers. The
Data Link task calls `gmp_dev_dl_dispatch_rx()`; it does not call a private
Scope dispatcher. The desktop configures and arms acquisitions over command
`0x60`.

Suite applications may use `CTL_DSA_DL_SCOPE_DEFINE_USER()`. It provides
`user_init_dl_scope()`, `user_dl_scope_facility()`, and
`user_step_dl_scope()` so the application can explicitly append the facility
without accessing adapter internals. `user_dispatch_dl_scope()` remains only
as a compatibility entry point for older applications.

## Ownership and debugging

`ctl_dsa_dl_scope_t` is deliberately non-opaque so CCS Expressions can inspect
capture state when diagnosing a target. This visibility does not transfer
configuration ownership to `user_main`: normal configuration is performed by
the host. The standard macro leaves `user_dl_scope` and
`user_dl_scope_storage` as global debugger-visible symbols. The enclosing
`gmp_datalink_t.service_run_count` reports execution of
the whole Data Link service task; no separate user-level
`datalink_task_runs` variable is required.
