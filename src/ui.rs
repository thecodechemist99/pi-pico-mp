use embedded_graphics::{
    prelude::*,
    pixelcolor::{
        BinaryColor,
        PixelColor,
    },
    primitives::{
        Rectangle,
        PrimitiveStyle,
    },
    text::Text,
    Drawable,
};

#[non_exhaustive]
pub enum State {
    Idle,
    Running,
    SetTemp,
    Menu,
    Cal,
}

struct Pages {
    
}

trait Container: Drawable + Dimensions
{
    type Color: PixelColor;
    type Output;

    fn draw<D>(&self, target: &mut D) -> Result<<Self as Container>::Output, D::Error>
    where
        D: DrawTarget<Color = <Self as Container>::Color>;

    fn bounding_box(&self) -> Rectangle;
}

struct UI<'a, C> {
    state: State,
    page_index: u8,
    pages: &'a Pages, // TODO: Define type for pages
    size: Size,
    bg_color: C,
}

impl<C> Container for UI<'_, C>
where
    Self: Container,
    C: PixelColor + From<BinaryColor>,
{
    type Color = C;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<<Self as Container>::Output, D::Error>
    where
        D: DrawTarget<Color = <Self as Container>::Color>
    {
        // for child in self.pages.iter() {
        //     match child.draw() {
        //         Err(e) => return Err(e),
        //         _ => (),
        //     }
        // }
        Ok(())
    }

    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::zero(), self.size)
    }

}

struct Page<C> {
    children: Pages, // TODO: Define type for children
    size: Size,
    bg_color: C,
}

impl<C> Page<C> {
    fn builder() -> PageBuilder {
        PageBuilder::new()
    }
}

impl<C> Container for Page<C>
where
    Self: Container,
    C: PixelColor + From<BinaryColor>,
{
    type Color = C;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<<Self as Container>::Output, D::Error>
    where
        D: DrawTarget<Color = <Self as Container>::Color>
    {
        // for child in self.pages.iter() {
        //     match child.draw() {
        //         Err(e) => return Err(e),
        //         _ => (),
        //     }
        // }
        Ok(())
    }

    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::zero(), self.size)
    }

}

struct PageBuilder<C> {

}

impl<C> PageBuilder<C> {
    fn new() -> Self {
        Self {
            
        }
    }

    fn build(self) -> Page<C> {
        let Self { } = self;
        Page { }
    }
}




// UI
// -> List of pages
// -> Interface to select pages
// -> Interface to set non-static content
// -> Keeps state

// Page
// -> Contains containers which will be displayed at once

// Container
// -> Contains display elements with relative positions
// -> Nestable

// Display element => embedded_graphics::Drawable
// -> Element which is displayed at a specific position on the screen
// -> Eg. text, images, graphic elements
