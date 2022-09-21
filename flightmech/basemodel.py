from pydantic import BaseModel


class FlightMechBaseModel(BaseModel):
    class Config:
        arbitrary_types_allowed = True
