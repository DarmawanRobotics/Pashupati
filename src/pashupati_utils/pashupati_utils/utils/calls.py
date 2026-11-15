"""Asynchronous service calls whose answer comes back as (ok, message)."""

from __future__ import annotations

from pashupati_utils.utils.params import outcome


def call_service(client, request, done) -> bool:
    """Call a service, done(ok, message, result) once it answers; False when it is unavailable."""
    if not client.service_is_ready():
        return False

    def answered(future):
        try:
            result = future.result()
        except Exception as error:  # the server raised: report it, never into the executor
            done(False, f'{client.srv_name} failed: {error}', None)
            return
        ok, message = outcome(result)
        done(ok, message, result)

    client.call_async(request).add_done_callback(answered)
    return True
