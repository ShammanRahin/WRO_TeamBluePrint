"""
FastAPI entrypoint — Phase 3 (Backend).

On startup: opens one AsyncClient to Supabase (shared by every route via
app.state.supabase) and subscribes it to the `office-updates` broadcast
topic so the alert engine runs event-driven, never on a poll loop.
"""

from __future__ import annotations

import logging
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from supabase import AsyncClient, acreate_client

from .realtime_listener import start_listener
from .routes import room, status, usage

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("office-monitor-backend")


@asynccontextmanager
async def lifespan(app: FastAPI):
    supabase_url = os.environ["SUPABASE_URL"]
    # Service role key: the backend evaluates/raises/clears alerts directly
    # against Postgres and needs to bypass RLS to do so reliably.
    supabase_key = os.environ["SUPABASE_SERVICE_KEY"]

    supabase: AsyncClient = await acreate_client(supabase_url, supabase_key)
    app.state.supabase = supabase

    channel = await start_listener(supabase)

    try:
        yield
    finally:
        await channel.unsubscribe()


app = FastAPI(title="Office Monitor Backend", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=os.environ.get("CORS_ORIGINS", "*").split(","),
    allow_methods=["GET"],
    allow_headers=["*"],
)

app.include_router(status.router)
app.include_router(room.router)
app.include_router(usage.router)


@app.get("/health")
async def health():
    return {"ok": True}
